#!/usr/bin/env python3
"""全流程与分步脚本：plan_1_receipt.py / plan_2_pick.py / plan_3_place.py。"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path
from typing import Any

import yaml


DEFAULT_PERCEPTION_URL = os.getenv(
    "PERCEPTION_URL", "http://172.20.10.5:8083"
).rstrip("/")
DEFAULT_SKU_URL = os.getenv("SKU_API_URL", "http://172.20.10.5:25540").rstrip("/")
DEFAULT_NAVIGATION_URL = os.getenv(
    "NAVIGATION_URL", "http://172.20.10.6:8081"
).rstrip("/")
DEFAULT_RGBD_URL = os.getenv("CAMERA_RGBD_URL", "http://127.0.0.1:8085").rstrip("/")
DEFAULT_POSE_URL = os.getenv("POSE_URL", "http://127.0.0.1:8099").rstrip("/")
DEFAULT_PICK_POSE_URL = os.getenv("PICK_POSE_URL", "http://172.20.10.5:18084").rstrip("/")
DEFAULT_TASK_TYPE = os.getenv("PICK_TASK_TYPE", "SORTING")
DEFAULT_SNAPSHOT_DIR = Path(
    os.getenv("PLAN_SNAPSHOT_DIR", str(Path(__file__).resolve().parent / "snapshots"))
)
DEFAULT_STATE_PATH = Path(
    os.getenv("PLAN_STATE_PATH", str(DEFAULT_SNAPSHOT_DIR / "plan_last.json"))
)
DEFAULT_ENDPOINT_CONFIG = Path(__file__).resolve().parent / "config" / "service_endpoints.yaml"


def load_endpoint_profile(config_path: Path, requested_profile: str = "") -> tuple[str, dict[str, str]]:
    """Load one endpoint profile. Environment and CLI URL options override it later."""

    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"端点配置文件不存在: {config_path}") from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f"端点配置 YAML 无效: {config_path}: {exc}") from exc

    if not isinstance(payload, dict):
        raise RuntimeError(f"端点配置顶层必须是对象: {config_path}")
    profiles = payload.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise RuntimeError(f"端点配置缺少 profiles: {config_path}")

    profile_name = (
        requested_profile.strip()
        or os.getenv("PLAN_PROFILE", "").strip()
        or str(payload.get("active_profile") or "").strip()
    )
    if not profile_name:
        raise RuntimeError("未指定端点 profile，且 active_profile 为空")
    profile = profiles.get(profile_name)
    if not isinstance(profile, dict):
        available = ", ".join(sorted(str(name) for name in profiles))
        raise RuntimeError(f"未知端点 profile={profile_name!r}，可用值: {available}")

    keys = (
        "perception_url",
        "sku_url",
        "navigation_url",
        "rgbd_url",
        "pose_url",
        "pick_pose_url",
    )
    endpoints: dict[str, str] = {}
    for key in keys:
        value = profile.get(key)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(f"profile={profile_name!r} 缺少有效 {key}")
        endpoints[key] = value.strip().rstrip("/")
    return profile_name, endpoints


def endpoint_default(
    profile: dict[str, str], key: str, environment_name: str, legacy_default: str
) -> str:
    """Resolve endpoint as environment > YAML profile > legacy code default."""

    return (
        os.getenv(environment_name, "").strip()
        or profile.get(key, "").strip()
        or legacy_default
    ).rstrip("/")

_NO_PROXY_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


INITIAL_STATION = "mark_8"  # 完整任务开始和结束位置
RECEIPT_STATION = "mark_0"
DELIVERY_STATION = RECEIPT_STATION  # 小票识别与物品放置共用同一导航点
RETREAT_FRONT = "mark_6"  # H1_F 正面后退点
RETREAT_BACK = "mark_7"  # H1_B 反面后退点
LEFT_MAX_COLUMN = 3  # C01–C03 左手深度相机，C04 及以后右手
POSE_RECEIPT = "RECEIPT_VIEW"
POSE_PLACE = "DELIVERY_TABLE_PLACE_READY"
_SLOT_FACE_RE = re.compile(r"^H[12]_([FB])(?:_|$)")
_SLOT_COLUMN_RE = re.compile(r"^H[12]_[FB]_L[1-5]_C(\d{2})$")
_SLOT_LEVEL_RE = re.compile(r"^H[12]_[FB]_L([1-5])_C\d{2}$")
_FRONT_STATIONS = {"MARK_4", "MARK_5", "LEFT_GRASP", "RIGHT_GRASP"}
_BACK_STATIONS = {"MARK_2", "MARK_3", "SHELF_BACK_LEFT", "SHELF_BACK_RIGHT"}
_LEFT_STATIONS = {"MARK_5", "MARK_2", "LEFT_GRASP", "SHELF_BACK_LEFT"}
_RIGHT_STATIONS = {"MARK_4", "MARK_3", "RIGHT_GRASP", "SHELF_BACK_RIGHT"}


def wait_nav_ready(navigation_url: str, *, timeout: float = 60.0) -> None:
    deadline = time.time() + timeout
    last = "UNREACHABLE"
    while time.time() < deadline:
        try:
            payload = _request_json(
                "GET",
                f"{navigation_url}/navigation/health",
                timeout=3.0,
            )
            last = str((payload or {}).get("status") or payload)
            if isinstance(payload, dict) and payload.get("status") == "READY":
                print("      导航 READY")
                return
        except RuntimeError as exc:
            last = str(exc)
        time.sleep(2.0)
    raise RuntimeError(f"导航未就绪，最后状态: {last}")


def wait_pose_ready(pose_url: str, *, timeout: float = 60.0) -> None:
    deadline = time.time() + timeout
    last = "UNREACHABLE"
    while time.time() < deadline:
        try:
            payload = _request_json(
                "GET",
                f"{pose_url}/pose/health",
                timeout=3.0,
            )
            last = str((payload or {}).get("status") or payload)
            if isinstance(payload, dict) and payload.get("status") == "READY":
                print("      机械臂姿态 READY")
                return
        except RuntimeError as exc:
            last = str(exc)
        time.sleep(2.0)
    raise RuntimeError(f"机械臂姿态未就绪，最后状态: {last}")


def _request_json(
    method: str,
    url: str,
    *,
    timeout: float,
    payload: dict[str, Any] | None = None,
    headers: dict[str, str] | None = None,
) -> Any:
    req_headers = {"Accept": "application/json"}
    data = None
    if payload is not None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req_headers["Content-Type"] = "application/json"
    if headers:
        req_headers.update(headers)

    request = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers=req_headers,
    )
    try:
        with _NO_PROXY_OPENER.open(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{method} {url} -> HTTP {exc.code}: {body}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"{method} {url} 连接失败: {exc.reason}") from exc

    if not raw.strip():
        return None
    return json.loads(raw)


def parse_receipt(perception_url: str, timeout: float) -> list[str]:
    """步骤 1：POST /perception/parse → product_names。"""

    payload = _request_json(
        "POST",
        f"{perception_url}/perception/parse",
        timeout=timeout,
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"parse 响应不是对象: {payload!r}")
    if "error" in payload:
        raise RuntimeError(f"parse 失败: {json.dumps(payload, ensure_ascii=False)}")

    names = payload.get("product_names")
    if not isinstance(names, list) or not all(
        isinstance(n, str) and n.strip() for n in names
    ):
        raise RuntimeError(f"parse 缺少有效 product_names: {payload!r}")
    return [name.strip() for name in names]


def search_by_name(sku_url: str, name: str, timeout: float) -> dict[str, Any]:
    """步骤 2：GET /sku/search_by_name?name=... → locations 等。"""

    query = urllib.parse.urlencode({"name": name})
    payload = _request_json(
        "GET",
        f"{sku_url}/sku/search_by_name?{query}",
        timeout=timeout,
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"SKU 响应不是对象: {payload!r}")
    if "error_code" in payload:
        raise RuntimeError(f"SKU 查询失败 ({name}): {payload}")
    return payload


def navigate_to(
    navigation_url: str,
    target_id: str,
    *,
    timeout: float,
    idempotency_key: str,
) -> dict[str, Any]:
    """步骤 3：POST /navigation/navigate → {"status":"SUCCEEDED"}。"""

    payload = _request_json(
        "POST",
        f"{navigation_url}/navigation/navigate",
        timeout=timeout,
        payload={"target_id": target_id},
        headers={"Idempotency-Key": idempotency_key},
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"导航响应不是对象: {payload!r}")
    if payload.get("status") != "SUCCEEDED":
        raise RuntimeError(
            f"导航未成功 ({target_id}): {json.dumps(payload, ensure_ascii=False)}"
        )
    return payload


def go_to(
    navigation_url: str,
    target_id: str,
    *,
    timeout: float,
    run_id: str,
    role: str,
    nav_results: list[dict[str, Any]],
    label: str,
) -> dict[str, Any]:
    """导航到一点并记入 nav_results；每次用新的 Idempotency-Key。"""

    key = f"plan-{run_id}:nav-{role}-{target_id}-{uuid.uuid4().hex[:8]}"
    started = time.time()
    response = navigate_to(
        navigation_url,
        target_id,
        timeout=timeout,
        idempotency_key=key,
    )
    elapsed = round(time.time() - started, 1)
    nav_results.append(
        {
            "target_id": target_id,
            "status": response.get("status"),
            "elapsed_seconds": elapsed,
            "idempotency_key": key,
            "role": role,
        }
    )
    print(f"      完成 {label} {target_id} ({elapsed}s) {response}")
    return response


def navigate_retreat_then_place(
    navigation_url: str,
    sku_location: str,
    *,
    timeout: float,
    run_id: str,
    nav_results: list[dict[str, Any]],
    pose_url: str,
    pose_timeout: float,
    skip_pose: bool,
    pose_results: list[dict[str, Any]],
    skip_release: bool = False,
    execute_release_motion: bool = False,
    release_timeout: float = 300.0,
    release_results: list[dict[str, Any]] | None = None,
) -> None:
    """抓取后：F→mark_6 / B→mark_7，再到 mark_0，放置位姿，然后 release。"""

    retreat = retreat_station(sku_location)
    print(
        f"[9] 抓取后回后退点再去放置点 {DELIVERY_STATION} "
        f"（F→{RETREAT_FRONT} / B→{RETREAT_BACK}）"
    )
    if retreat:
        print(f"      导航到后退点 {retreat}")
        go_to(
            navigation_url,
            retreat,
            timeout=timeout,
            run_id=run_id,
            role="retreat_after_pick",
            nav_results=nav_results,
            label="后退点",
        )
    else:
        print(f"      货位 {sku_location} 无法判断后退点，直接去放置点")
    print(f"      导航到放置点 {DELIVERY_STATION}")
    go_to(
        navigation_url,
        DELIVERY_STATION,
        timeout=timeout,
        run_id=run_id,
        role="delivery",
        nav_results=nav_results,
        label="放置点",
    )
    if skip_pose:
        print(f"      跳过放置位姿 {POSE_PLACE} 和 release")
        return
    print(f"[10] 放置位姿 {POSE_PLACE}")
    pose_info = move_to_place_pose(
        pose_url,
        timeout=pose_timeout,
        run_id=run_id,
    )
    pose_results.append({**pose_info, "location": sku_location})
    if skip_release:
        print("      跳过 /manipulation/release")
        return
    hand = hand_camera_for_location(sku_location)
    key = f"plan-{run_id}:release-{hand}-{uuid.uuid4().hex[:8]}"
    mode = "真放" if execute_release_motion else "只检查不松爪"
    print(
        f"[11] POST {pose_url}/manipulation/release "
        f"hand={hand.upper()} execute={execute_release_motion} ({mode})"
    )
    print(f"      key={key}")
    started = time.time()
    response = execute_release(
        pose_url,
        hand=hand,
        timeout=release_timeout,
        idempotency_key=key,
        execute=execute_release_motion,
    )
    elapsed = round(time.time() - started, 1)
    record = {
        "hand": hand.upper(),
        "execute": execute_release_motion,
        "idempotency_key": key,
        "elapsed_seconds": elapsed,
        "status": response.get("status"),
        "location": sku_location,
    }
    if release_results is not None:
        release_results.append(record)
    print(
        f"      release 完成 ({elapsed}s) status={response.get('status')}"
    )


def lookup_products(
    sku_url: str, names: list[str], sku_timeout: float
) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    for name in names:
        product = search_by_name(sku_url, name, sku_timeout)
        locations = product.get("locations") or []
        primary = locations[0] if locations else None
        item = {
            "product_name": product.get("name", name),
            "sku_id": product.get("sku_id"),
            "locations": locations,
            "primary_location": primary,
            "images": product.get("images") or [],
        }
        results.append(item)
        print(f"      {item['product_name']} -> {primary} ({locations})")
    return results


def product_jobs(
    results: list[dict[str, Any]],
    *,
    nav_target: str | None = None,
    product_name: str | None = None,
) -> list[dict[str, str]]:
    """按小票顺序一件一轮，同货位也不合并。"""

    jobs: list[dict[str, str]] = []
    for item in results:
        name = item.get("product_name")
        loc = item.get("primary_location")
        if not isinstance(name, str) or not name.strip():
            raise RuntimeError(f"商品缺少名称: {item!r}")
        if not isinstance(loc, str) or not loc.strip():
            raise RuntimeError(f"商品缺少货位: {name}")
        jobs.append(
            {
                "product_name": name.strip(),
                "location": loc.strip().upper(),
            }
        )
    override_name = product_name.strip() if product_name else None
    override_loc = nav_target.strip().upper() if nav_target else None
    if override_name:
        matched = [job for job in jobs if job["product_name"] == override_name]
        if matched:
            jobs = matched
        elif override_loc:
            jobs = [{"product_name": override_name, "location": override_loc}]
        elif jobs:
            raise RuntimeError(f"小票中没有商品 {override_name}")
        else:
            jobs = []
    if override_loc:
        matching = [job for job in jobs if job["location"] == override_loc]
        if matching:
            jobs = matching
        elif jobs:
            jobs = [{**jobs[0], "location": override_loc}]
        else:
            jobs = [
                {
                    "product_name": override_name or "item",
                    "location": override_loc,
                }
            ]
    if not jobs:
        raise RuntimeError("没有可执行商品")
    return jobs


def current_job(
    state: dict[str, Any] | None,
    *,
    nav_target: str | None = None,
    product_name: str | None = None,
    results: list[dict[str, Any]] | None = None,
) -> tuple[int, dict[str, str]]:
    jobs = list((state or {}).get("jobs") or [])
    if not jobs:
        jobs = product_jobs(
            list(results or (state or {}).get("products") or []),
            nav_target=nav_target,
            product_name=product_name,
        )
    idx = int((state or {}).get("pending_index") or 0)
    if idx < 0 or idx >= len(jobs):
        raise RuntimeError("没有待抓取商品")
    job = dict(jobs[idx])
    if product_name:
        job["product_name"] = product_name.strip()
    if nav_target:
        job["location"] = nav_target.strip().upper()
    return idx, job


def retreat_station(target_id: str) -> str | None:
    """F 货位 / 正面抓取点 → mark_6；B 货位 / 反面抓取点 → mark_7。"""

    token = target_id.strip().upper()
    match = _SLOT_FACE_RE.match(token)
    if match:
        return RETREAT_FRONT if match.group(1) == "F" else RETREAT_BACK
    if token in _FRONT_STATIONS:
        return RETREAT_FRONT
    if token in _BACK_STATIONS:
        return RETREAT_BACK
    return None


def with_retreat_waypoints(targets: list[str]) -> list[tuple[str, str]]:
    """每个货位前插入对应后退点；连续相同点只走一次。"""

    waypoints: list[tuple[str, str]] = []
    for target in targets:
        retreat = retreat_station(target)
        if retreat and (not waypoints or waypoints[-1][0] != retreat):
            waypoints.append((retreat, "retreat"))
        elif retreat:
            print(f"      已在后退点 {retreat}，跳过")
        if not waypoints or waypoints[-1][0] != target:
            waypoints.append((target, "sku_location"))
        else:
            print(f"      跳过重复点 {target}")
    return waypoints


def navigate_to_sku_location(
    navigation_url: str,
    location: str,
    *,
    timeout: float,
    run_id: str,
    nav_results: list[dict[str, Any]],
    round_index: int = 1,
    product_name: str = "",
) -> None:
    """只去这一件商品的后退点 + 货位。"""

    waypoints = with_retreat_waypoints([location])
    label_name = f" {product_name}" if product_name else ""
    print(
        f"[4] 第{round_index}件{label_name} 货位导航"
        f"（F→{RETREAT_FRONT} / B→{RETREAT_BACK}）"
        f" {[target_id for target_id, _ in waypoints]}"
    )
    for target_id, role in waypoints:
        go_to(
            navigation_url,
            target_id,
            timeout=timeout,
            run_id=run_id,
            role=role if role != "sku_location" else "sku_location",
            nav_results=nav_results,
            label="后退点" if role == "retreat" else "货位",
        )


def hand_camera_for_location(location: str) -> str:
    """C01–C03 → left，C04 及以后 → right。"""

    token = location.strip().upper()
    match = _SLOT_COLUMN_RE.fullmatch(token)
    if match:
        column = int(match.group(1))
        return "left" if column <= LEFT_MAX_COLUMN else "right"
    if token in _LEFT_STATIONS:
        return "left"
    if token in _RIGHT_STATIONS:
        return "right"
    raise RuntimeError(f"无法判断货位对应左右手相机: {location}")


def shelf_pose_id(location: str) -> str:
    """H*_L1_* → level_1，L2 → level_2，L3 → level_3，L4 → level_4。"""

    token = location.strip().upper()
    match = _SLOT_LEVEL_RE.fullmatch(token)
    if not match:
        raise RuntimeError(f"无法从货位解析层号: {location}")
    level = int(match.group(1))
    if level not in (1, 2, 3, 4):
        raise RuntimeError(f"没有第{level}层拍照位姿，货位={location}")
    return f"level_{level}"


def prepare_pose(
    pose_url: str,
    pose_id: str,
    *,
    timeout: float,
    idempotency_key: str,
) -> dict[str, Any]:
    """POST /pose/prepare，每次必须用新的 Idempotency-Key。"""

    payload = _request_json(
        "POST",
        f"{pose_url}/pose/prepare",
        timeout=timeout,
        payload={"pose_id": pose_id},
        headers={"Idempotency-Key": idempotency_key},
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"姿态响应不是对象: {payload!r}")
    if payload.get("status") != "SUCCEEDED":
        raise RuntimeError(
            f"姿态未成功 ({pose_id}): {json.dumps(payload, ensure_ascii=False)}"
        )
    return payload


def move_to_photo_pose(
    pose_url: str,
    location: str,
    *,
    timeout: float,
    run_id: str,
) -> dict[str, Any]:
    pose_id = shelf_pose_id(location)
    key = f"plan-{run_id}:pose-{pose_id}-{uuid.uuid4().hex[:8]}"
    print(f"      POST {pose_url}/pose/prepare pose_id={pose_id} key={key}")
    started = time.time()
    response = prepare_pose(
        pose_url,
        pose_id,
        timeout=timeout,
        idempotency_key=key,
    )
    elapsed = round(time.time() - started, 1)
    print(f"      到位 {pose_id} ({elapsed}s) {response}")
    return {
        "pose_id": pose_id,
        "status": response.get("status"),
        "elapsed_seconds": elapsed,
        "idempotency_key": key,
    }


def move_to_receipt_pose(
    pose_url: str,
    *,
    timeout: float,
    run_id: str,
) -> dict[str, Any]:
    key = f"plan-{run_id}:pose-{POSE_RECEIPT}-{uuid.uuid4().hex[:8]}"
    print(f"      POST {pose_url}/pose/prepare pose_id={POSE_RECEIPT} key={key}")
    started = time.time()
    response = prepare_pose(
        pose_url,
        POSE_RECEIPT,
        timeout=timeout,
        idempotency_key=key,
    )
    elapsed = round(time.time() - started, 1)
    print(f"      到位 {POSE_RECEIPT} ({elapsed}s) {response}")
    return {
        "pose_id": POSE_RECEIPT,
        "status": response.get("status"),
        "elapsed_seconds": elapsed,
        "idempotency_key": key,
        "role": "receipt",
    }


def move_to_place_pose(
    pose_url: str,
    *,
    timeout: float,
    run_id: str,
) -> dict[str, Any]:
    key = f"plan-{run_id}:pose-{POSE_PLACE}-{uuid.uuid4().hex[:8]}"
    print(f"      POST {pose_url}/pose/prepare pose_id={POSE_PLACE} key={key}")
    started = time.time()
    response = prepare_pose(
        pose_url,
        POSE_PLACE,
        timeout=timeout,
        idempotency_key=key,
    )
    elapsed = round(time.time() - started, 1)
    print(f"      到位 {POSE_PLACE} ({elapsed}s) {response}")
    return {
        "pose_id": POSE_PLACE,
        "status": response.get("status"),
        "elapsed_seconds": elapsed,
        "idempotency_key": key,
        "role": "place",
    }


def capture_hand_rgbd(
    rgbd_url: str, camera: str, timeout: float
) -> dict[str, Any]:
    payload = _request_json(
        "GET",
        f"{rgbd_url}/camera/rgbd?camera={camera}&format=json",
        timeout=timeout,
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"RGB-D 响应不是对象: {payload!r}")
    if "error" in payload:
        raise RuntimeError(
            f"RGB-D 失败: {json.dumps(payload, ensure_ascii=False)}"
        )
    if "rgb_png_b64" not in payload or "depth_png_b64" not in payload:
        raise RuntimeError("RGB-D 响应缺少 rgb_png_b64 / depth_png_b64")
    return payload


def save_rgbd_capture(payload: dict[str, Any], out_dir: Path) -> dict[str, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = out_dir / "rgb.png"
    depth_path = out_dir / "depth.png"
    meta_path = out_dir / "camera.json"
    rgb_path.write_bytes(base64.b64decode(payload["rgb_png_b64"]))
    depth_path.write_bytes(base64.b64decode(payload["depth_png_b64"]))
    meta = {
        key: value
        for key, value in payload.items()
        if key not in {"rgb_png_b64", "depth_png_b64"}
    }
    meta_path.write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {
        "rgb": str(rgb_path),
        "depth": str(depth_path),
        "camera": str(meta_path),
    }


def product_names_for_location(
    results: list[dict[str, Any]], location: str, override: str | None
) -> list[str]:
    if override:
        return [override]
    names: list[str] = []
    token = location.strip().upper()
    for item in results:
        loc = str(item.get("primary_location") or "").strip().upper()
        name = item.get("product_name")
        if loc == token and isinstance(name, str) and name.strip():
            names.append(name.strip())
    return names


def save_plan_state(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"      状态已写入 {path}")


def load_plan_state(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(
            f"没有状态文件 {path}，请先跑步骤 1 或指定 --nav-target"
        )
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"状态文件不是对象: {path}")
    return payload


def resolve_location(
    nav_target: str | None, state: dict[str, Any] | None
) -> str:
    if nav_target:
        return nav_target.strip().upper()
    if state:
        current = state.get("current_location")
        if isinstance(current, str) and current.strip():
            return current.strip().upper()
        locations = state.get("locations") or []
        if locations:
            return str(locations[-1]).strip().upper()
    raise RuntimeError("需要 --nav-target，或先跑步骤 1 生成状态文件")


def dump_summary(summary: dict[str, Any]) -> None:
    print("\n=== 汇总 ===")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def locate_pick(
    perception_url: str,
    *,
    task_type: str,
    product_name: str,
    hand: str,
    timeout: float,
    image_path: Path | None = None,
) -> dict[str, Any]:
    """POST /perception/pick/locate，传入刚拍的 RGB 图。"""

    request_payload: dict[str, Any] = {
        "task_type": task_type,
        "product_name": product_name,
        "hand": hand,
    }
    if image_path is not None:
        raw = image_path.read_bytes()
        if not raw:
            raise RuntimeError(f"locate 图片为空: {image_path}")
        request_payload["image_name"] = image_path.name
        request_payload["image_base64"] = base64.b64encode(raw).decode("ascii")

    payload = _request_json(
        "POST",
        f"{perception_url}/perception/pick/locate",
        timeout=timeout,
        payload=request_payload,
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"locate 响应不是对象: {payload!r}")
    if "error" in payload or payload.get("error_code"):
        raise RuntimeError(
            f"locate 失败: {json.dumps(payload, ensure_ascii=False)}"
        )
    if "bbox" not in payload and "mask" not in payload:
        raise RuntimeError(f"locate 缺少 bbox/mask: {payload!r}")
    return payload


def save_locate_mask(
    locate: dict[str, Any],
    out_path: Path,
    *,
    width: int,
    height: int,
) -> Path:
    raw = locate.get("mask")
    if isinstance(raw, str) and raw.strip():
        try:
            data = base64.b64decode(raw, validate=False)
        except Exception:
            data = b""
        if data.startswith(b"\x89PNG") or data.startswith(b"\xff\xd8"):
            out_path.write_bytes(data)
            return out_path

    bbox = locate.get("bbox")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise RuntimeError("locate 没有可用 mask，也无法用 bbox 生成")
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("locate 未给图像 mask，本地又没有 OpenCV 可从 bbox 生成") from exc

    x1, y1, x2, y2 = (float(v) for v in bbox)
    px1 = max(0, min(width - 1, int(round((x1 - 1) / 999.0 * (width - 1)))))
    py1 = max(0, min(height - 1, int(round((y1 - 1) / 999.0 * (height - 1)))))
    px2 = max(0, min(width, int(round((x2 - 1) / 999.0 * (width - 1))) + 1))
    py2 = max(0, min(height, int(round((y2 - 1) / 999.0 * (height - 1))) + 1))
    if px2 <= px1:
        px2 = min(width, px1 + 1)
    if py2 <= py1:
        py2 = min(height, py1 + 1)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[py1:py2, px1:px2] = 255
    if not cv2.imwrite(str(out_path), mask):
        raise RuntimeError(f"mask 保存失败: {out_path}")
    return out_path


def _multipart_body(
    fields: dict[str, str],
    files: dict[str, tuple[str, bytes, str]],
) -> tuple[bytes, str]:
    boundary = "----PlanBoundary" + uuid.uuid4().hex
    chunks: list[bytes] = []
    for name, value in fields.items():
        chunks.append(
            (
                f"--{boundary}\r\n"
                f'Content-Disposition: form-data; name="{name}"\r\n\r\n'
                f"{value}\r\n"
            ).encode("utf-8")
        )
    for name, (filename, data, content_type) in files.items():
        chunks.append(
            (
                f"--{boundary}\r\n"
                f'Content-Disposition: form-data; name="{name}"; '
                f'filename="{filename}"\r\n'
                f"Content-Type: {content_type}\r\n\r\n"
            ).encode("utf-8")
        )
        chunks.append(data)
        chunks.append(b"\r\n")
    chunks.append(f"--{boundary}--\r\n".encode("utf-8"))
    return b"".join(chunks), boundary


def estimate_pick_pose(
    pick_pose_url: str,
    *,
    rgb_path: Path,
    depth_path: Path,
    camera_path: Path,
    mask_path: Path,
    product_name: str | None,
    timeout: float,
) -> dict[str, Any]:
    """POST /infer（GenPose2），传入 rgb / depth / camera / mask 文件。"""

    fields: dict[str, str] = {}
    if product_name:
        fields["sam3_prompt"] = product_name
    body, boundary = _multipart_body(
        fields,
        {
            "rgb": (rgb_path.name, rgb_path.read_bytes(), "image/png"),
            "depth": (depth_path.name, depth_path.read_bytes(), "image/png"),
            "camera": (camera_path.name, camera_path.read_bytes(), "application/json"),
            "mask": (mask_path.name, mask_path.read_bytes(), "image/png"),
        },
    )
    request = urllib.request.Request(
        f"{pick_pose_url}/infer",
        data=body,
        method="POST",
        headers={
            "Accept": "application/json",
            "Content-Type": f"multipart/form-data; boundary={boundary}",
        },
    )
    try:
        with _NO_PROXY_OPENER.open(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        err_body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(
            f"POST {pick_pose_url}/infer -> HTTP {exc.code}: {err_body}"
        ) from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(
            f"POST {pick_pose_url}/infer 连接失败: {exc.reason}"
        ) from exc
    if not raw.strip():
        raise RuntimeError("pick_pose 响应为空")
    payload = json.loads(raw)
    if not isinstance(payload, dict):
        raise RuntimeError(f"pick_pose 响应不是对象: {payload!r}")
    if payload.get("error") or payload.get("error_code"):
        raise RuntimeError(
            f"pick_pose 失败: {json.dumps(payload, ensure_ascii=False)}"
        )
    pose = payload.get("pose")
    if not isinstance(pose, list) or len(pose) != 6:
        pose = payload.get("xyzrxryrz")
    if not isinstance(pose, list) or len(pose) != 6:
        raise RuntimeError(f"pick_pose 缺少 6D pose: {payload!r}")
    payload["pose"] = [float(value) for value in pose]
    return payload


def genpose_to_grasp_xyz(payload: dict[str, Any]) -> list[float]:
    """GenPose 旋转矩阵 → grasp 用的 [x, y, z, rx, ry, rz]（scipy xyz，mm_rad）。

    字段虽写 rotation_order=zyx，但 detections_pem.R 与 xyzrxryrz[3:] 对得上的
    是 xyz：R == Rotation.from_euler("xyz", rx, ry, rz)。按 zyx 解会把绿色轴
    弄歪（本次 38.7°），用矩阵再出 xyz 才和真实 R 一致。
    """

    try:
        from scipy.spatial.transform import Rotation
    except ImportError as exc:
        raise RuntimeError("转换欧拉角需要 scipy，请用 conda env hand_eye_calib") from exc

    xyz_mm = payload.get("xyz_mm")
    if not (isinstance(xyz_mm, list) and len(xyz_mm) == 3):
        raw = payload.get("xyzrxryrz") or payload.get("pose")
        if not (isinstance(raw, list) and len(raw) >= 3):
            raise RuntimeError(f"GenPose 响应缺少 xyz: {payload!r}")
        xyz_mm = raw[:3]

    rotation = None
    detections = payload.get("detections_pem")
    if isinstance(detections, list) and detections:
        matrix = detections[0].get("R") if isinstance(detections[0], dict) else None
        if isinstance(matrix, list) and len(matrix) == 3:
            rotation = Rotation.from_matrix(matrix)

    if rotation is None:
        raw = payload.get("xyzrxryrz") or payload.get("pose")
        if not (isinstance(raw, list) and len(raw) == 6):
            raise RuntimeError(f"GenPose 响应缺少旋转: {payload!r}")
        rotation = Rotation.from_euler(
            "xyz", [float(value) for value in raw[3:]], degrees=False
        )

    euler_xyz = rotation.as_euler("xyz", degrees=False)
    return [
        float(xyz_mm[0]),
        float(xyz_mm[1]),
        float(xyz_mm[2]),
        float(euler_xyz[0]),
        float(euler_xyz[1]),
        float(euler_xyz[2]),
    ]


def execute_grasp(
    pose_url: str,
    *,
    pose: list[float],
    hand: str,
    timeout: float,
    idempotency_key: str,
    execute: bool,
) -> dict[str, Any]:
    """POST /manipulation/grasp。execute=false 只规划，true 才真抓。"""

    payload = _request_json(
        "POST",
        f"{pose_url}/manipulation/grasp",
        timeout=timeout,
        payload={
            "pose": [float(value) for value in pose],
            "hand": hand.upper(),
            "frame": "camera",
            "pose_unit": "mm_rad",
            "rotation_order": "xyz",
            "execute": execute,
        },
        headers={"Idempotency-Key": idempotency_key},
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"grasp 响应不是对象: {payload!r}")
    if payload.get("error_code") or payload.get("status") in {"FAILED", "UNREACHABLE"}:
        raise RuntimeError(
            f"grasp 失败: {json.dumps(payload, ensure_ascii=False)}"
        )
    return payload


def execute_release(
    pose_url: str,
    *,
    hand: str,
    timeout: float,
    idempotency_key: str,
    execute: bool,
) -> dict[str, Any]:
    """POST /manipulation/release。execute=false 只检查，true 才松爪放置。"""

    payload = _request_json(
        "POST",
        f"{pose_url}/manipulation/release",
        timeout=timeout,
        payload={
            "hand": hand.upper(),
            "execute": execute,
        },
        headers={"Idempotency-Key": idempotency_key},
    )
    if not isinstance(payload, dict):
        raise RuntimeError(f"release 响应不是对象: {payload!r}")
    if payload.get("error_code") or payload.get("status") in {"FAILED", "UNREACHABLE"}:
        raise RuntimeError(
            f"release 失败: {json.dumps(payload, ensure_ascii=False)}"
        )
    return payload


def pick_at_location(
    *,
    target_id: str,
    run_id: str,
    perception_url: str,
    rgbd_url: str,
    pose_url: str,
    pick_pose_url: str,
    pose_timeout: float,
    rgbd_timeout: float,
    locate_timeout: float,
    pick_pose_timeout: float,
    grasp_timeout: float,
    skip_pose: bool,
    capture: bool,
    locate: bool,
    grasp: bool,
    execute_grasp_motion: bool,
    results: list[dict[str, Any]],
    product_name: str | None,
    task_type: str,
    snapshot_dir: Path,
    pose_results: list[dict[str, Any]],
    capture_results: list[dict[str, Any]],
) -> None:
    """步骤 2：拍照位姿 → RGB-D → locate → pick_pose → grasp。假定已在货位。"""

    if skip_pose and not capture:
        print("      跳过拍照 / locate / grasp")
        return

    camera = hand_camera_for_location(target_id)
    print(f"[5] 货位 {target_id} → {camera} 臂深度相机")
    pose_info: dict[str, Any] | None = None
    if not skip_pose:
        pose_info = move_to_photo_pose(
            pose_url,
            target_id,
            timeout=pose_timeout,
            run_id=run_id,
        )
        pose_results.append({**pose_info, "location": target_id, "role": "photo"})
    if not capture:
        return

    print(f"      GET {rgbd_url}/camera/rgbd?camera={camera}")
    started = time.time()
    payload = capture_hand_rgbd(rgbd_url, camera, rgbd_timeout)
    elapsed = round(time.time() - started, 1)
    out_dir = snapshot_dir / f"plan-{run_id}" / f"{target_id}_{camera}"
    paths = save_rgbd_capture(payload, out_dir)
    capture_results.append(
        {
            "location": target_id,
            "camera": camera,
            "photo_pose": pose_info,
            "elapsed_seconds": elapsed,
            "depth_scale": payload.get("depth_scale"),
            "image_width": payload.get("image_width"),
            "image_height": payload.get("image_height"),
            "paths": paths,
        }
    )
    print(
        f"      完成拍照 ({elapsed}s) rgb={paths['rgb']} depth={paths['depth']}"
    )

    locates: list[dict[str, Any]] = []
    names = product_names_for_location(results, target_id, product_name)
    if locate and not names:
        print("      无商品名，跳过 locate / pick_pose")
    elif locate:
        width = int(payload.get("image_width") or 1280)
        height = int(payload.get("image_height") or 720)
        for name in names:
            print(
                f"[6] POST {perception_url}/perception/pick/locate "
                f"name={name} hand={camera} task={task_type} image={paths['rgb']}"
            )
            started = time.time()
            located = locate_pick(
                perception_url,
                task_type=task_type,
                product_name=name,
                hand=camera,
                timeout=locate_timeout,
                image_path=Path(paths["rgb"]),
            )
            elapsed = round(time.time() - started, 1)
            safe_name = "".join("_" if ch in "/\\" else ch for ch in name) or "item"
            mask_path = Path(out_dir) / f"mask_{safe_name}.png"
            save_locate_mask(located, mask_path, width=width, height=height)
            locate_path = Path(out_dir) / f"locate_{safe_name}.json"
            locate_meta = {
                field: value
                for field, value in located.items()
                if field != "mask"
            }
            locate_meta["mask_path"] = str(mask_path)
            locate_path.write_text(
                json.dumps(locate_meta, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"      locate 完成 ({elapsed}s) bbox={located.get('bbox')}")

            print(
                f"[7] POST {pick_pose_url}/infer name={name}"
            )
            started = time.time()
            pick = estimate_pick_pose(
                pick_pose_url,
                rgb_path=Path(paths["rgb"]),
                depth_path=Path(paths["depth"]),
                camera_path=Path(paths["camera"]),
                mask_path=mask_path,
                product_name=name,
                timeout=pick_pose_timeout,
            )
            elapsed = round(time.time() - started, 1)
            pick_path = Path(out_dir) / f"pick_pose_{safe_name}.json"
            pick_path.write_text(
                json.dumps(pick, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            locates.append(
                {
                    "product_name": name,
                    "bbox": located.get("bbox"),
                    "mask_path": str(mask_path),
                    "pose": pick.get("pose"),
                    "corners_mm": pick.get("corners_mm"),
                    "elapsed_seconds": elapsed,
                    "paths": {
                        "locate": str(locate_path),
                        "pick_pose": str(pick_path),
                    },
                }
            )
            print(f"      pick_pose 完成 ({elapsed}s) pose={pick.get('pose')}")

            if not grasp:
                continue
            grasp_pose = genpose_to_grasp_xyz(pick)
            print(
                f"      R→xyz rx,ry,rz {grasp_pose[3:]} "
                f"(raw {list(pick.get('pose') or [])[3:]})"
            )
            mode = "真抓" if execute_grasp_motion else "只规划不执行"
            key = f"plan-{run_id}:grasp-{camera}-{uuid.uuid4().hex[:8]}"
            print(
                f"[8] POST {pose_url}/manipulation/grasp "
                f"hand={camera.upper()} execute={execute_grasp_motion} ({mode})"
            )
            print(f"      key={key}")
            started = time.time()
            grasp_response = execute_grasp(
                pose_url,
                pose=grasp_pose,
                hand=camera,
                timeout=grasp_timeout,
                idempotency_key=key,
                execute=execute_grasp_motion,
            )
            elapsed = round(time.time() - started, 1)
            grasp_path = Path(out_dir) / f"grasp_{safe_name}.json"
            grasp_path.write_text(
                json.dumps(grasp_response, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            locates[-1]["grasp"] = {
                "hand": camera.upper(),
                "execute": execute_grasp_motion,
                "idempotency_key": key,
                "elapsed_seconds": elapsed,
                "status": grasp_response.get("status"),
                "reachable": grasp_response.get("reachable"),
                "pose_xyz": grasp_pose,
                "rotation_order": "xyz",
                "path": str(grasp_path),
            }
            locates[-1]["paths"]["grasp"] = str(grasp_path)
            print(
                f"      grasp 完成 ({elapsed}s) "
                f"status={grasp_response.get('status')} "
                f"reachable={grasp_response.get('reachable')}"
            )
    capture_results[-1]["locates"] = locates


def run_step1_receipt_to_shelf(
    *,
    perception_url: str,
    sku_url: str,
    navigation_url: str,
    pose_url: str,
    parse_timeout: float,
    sku_timeout: float,
    nav_timeout: float,
    pose_timeout: float,
    skip_receipt: bool,
    go_receipt_station: bool,
    skip_pose: bool,
    nav_target: str | None,
    settle_seconds: float,
    state_path: Path,
    product_name: str | None = None,
    next_item: bool = False,
) -> dict[str, Any]:
    """步骤 1：小票位 → 识别 → 第一件（或下一件）货位。"""

    run_id = uuid.uuid4().hex[:8]
    nav_results: list[dict[str, Any]] = []
    pose_results: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    state: dict[str, Any] | None = None
    jobs: list[dict[str, str]] = []
    pending_index = 0

    print("=== 步骤 1：小票位 → RECEIPT_VIEW → 识别 → 第一件货位 ===")
    print(f"[0] GET {navigation_url}/navigation/health")
    wait_nav_ready(navigation_url)

    if next_item:
        state = load_plan_state(state_path)
        results = list(state.get("products") or [])
        jobs = list(state.get("jobs") or product_jobs(results))
        pending_index = int(state.get("pending_index") or 0)
        run_id = str(state.get("run_id") or run_id)
        if pending_index >= len(jobs):
            raise RuntimeError("没有下一件商品")
        job = dict(jobs[pending_index])
        print(f"[3] GET {sku_url}/sku/search_by_name name={job['product_name']}")
        product = search_by_name(sku_url, job["product_name"], sku_timeout)
        locations = product.get("locations") or []
        if locations:
            old = job.get("location")
            job["location"] = str(locations[0]).strip().upper()
            jobs[pending_index] = job
            if pending_index < len(results) and isinstance(results[pending_index], dict):
                results[pending_index]["locations"] = list(locations)
                results[pending_index]["primary_location"] = job["location"]
            if old != job["location"]:
                print(f"      云上货位更新 {job['product_name']}: {old} → {job['location']}")
        print(
            f"      第{pending_index + 1}/{len(jobs)} 件 "
            f"{job['product_name']} @ {job['location']}"
        )
        navigate_to_sku_location(
            navigation_url,
            job["location"],
            timeout=nav_timeout,
            run_id=run_id,
            nav_results=nav_results,
            round_index=pending_index + 1,
            product_name=job["product_name"],
        )
        summary = {
            **state,
            "step": 1,
            "run_id": run_id,
            "products": results,
            "jobs": jobs,
            "pending_index": pending_index,
            "current_product": job["product_name"],
            "current_location": job["location"],
            "locations": [job["location"]],
            "navigation": nav_results,
        }
        save_plan_state(state_path, summary)
        print(f"      停在货位 {job['location']}，下一步跑 plan_2_pick.py")
        return summary

    if not skip_pose and (go_receipt_station or not skip_receipt):
        print(f"[0] GET {pose_url}/pose/health")
        wait_pose_ready(pose_url)

    if go_receipt_station:
        print(f"[1] 导航到小票位 {RECEIPT_STATION}")
        go_to(
            navigation_url,
            RECEIPT_STATION,
            timeout=nav_timeout,
            run_id=run_id,
            role="receipt",
            nav_results=nav_results,
            label="小票位",
        )
    if not skip_pose and (go_receipt_station or not skip_receipt):
        print(f"[1b] 小票识别位姿 {POSE_RECEIPT}")
        pose_results.append(
            move_to_receipt_pose(
                pose_url,
                timeout=pose_timeout,
                run_id=run_id,
            )
        )
        if settle_seconds > 0:
            print(f"      等待相机稳定 {settle_seconds:.0f}s")
            time.sleep(settle_seconds)

    if skip_receipt:
        print("[2] 跳过小票识别")
        print("[3] 跳过 SKU 查询")
    else:
        print(f"[2] POST {perception_url}/perception/parse")
        product_names = parse_receipt(perception_url, parse_timeout)
        print(f"      product_names = {product_names}")
        print(f"[3] GET {sku_url}/sku/search_by_name × {len(product_names)}")
        results = lookup_products(sku_url, product_names, sku_timeout)

    jobs = product_jobs(results, nav_target=nav_target, product_name=product_name)
    print(
        f"      小票 {len(jobs)} 件，本轮只去第 1 件 "
        f"{jobs[0]['product_name']} @ {jobs[0]['location']}"
    )
    if len(jobs) > 1:
        rest = ", ".join(
            f"{job['product_name']}@{job['location']}" for job in jobs[1:]
        )
        print(f"      其余稍后第二轮: {rest}")
    job = jobs[0]
    navigate_to_sku_location(
        navigation_url,
        job["location"],
        timeout=nav_timeout,
        run_id=run_id,
        nav_results=nav_results,
        round_index=1,
        product_name=job["product_name"],
    )

    summary = {
        "step": 1,
        "run_id": run_id,
        "products": results,
        "jobs": jobs,
        "pending_index": 0,
        "current_product": job["product_name"],
        "current_location": job["location"],
        "locations": [job["location"]],
        "navigation": nav_results,
        "poses": pose_results,
    }
    save_plan_state(state_path, summary)
    print(f"      停在货位 {job['location']}，下一步跑 plan_2_pick.py")
    return summary


def run_step2_pick(
    *,
    perception_url: str,
    rgbd_url: str,
    pose_url: str,
    pick_pose_url: str,
    pose_timeout: float,
    rgbd_timeout: float,
    locate_timeout: float,
    pick_pose_timeout: float,
    grasp_timeout: float,
    skip_pose: bool,
    capture: bool,
    locate: bool,
    grasp: bool,
    execute_grasp_motion: bool,
    nav_target: str | None,
    product_name: str | None,
    task_type: str,
    snapshot_dir: Path,
    state_path: Path,
) -> dict[str, Any]:
    """步骤 2：拍照 → locate → pick_pose → grasp。假定已在货位。"""

    print("=== 步骤 2：拍照 → locate → pick_pose → grasp ===")
    state: dict[str, Any] | None = None
    if state_path.is_file():
        state = load_plan_state(state_path)
    results = list((state or {}).get("products") or [])
    idx, job = current_job(
        state,
        nav_target=nav_target,
        product_name=product_name,
        results=results,
    )
    location = job["location"]
    name = job["product_name"]
    jobs = list((state or {}).get("jobs") or [job])
    run_id = str((state or {}).get("run_id") or uuid.uuid4().hex[:8])
    pose_results: list[dict[str, Any]] = []
    capture_results: list[dict[str, Any]] = []
    print(f"      本轮只抓第{idx + 1}/{len(jobs)} 件 {name} @ {location}")

    if locate and not name:
        raise RuntimeError("没有商品名：给 --product-name，或先跑步骤 1")

    if not skip_pose:
        print(f"[0] GET {pose_url}/pose/health")
        wait_pose_ready(pose_url)

    pick_at_location(
        target_id=location,
        run_id=run_id,
        perception_url=perception_url,
        rgbd_url=rgbd_url,
        pose_url=pose_url,
        pick_pose_url=pick_pose_url,
        pose_timeout=pose_timeout,
        rgbd_timeout=rgbd_timeout,
        locate_timeout=locate_timeout,
        pick_pose_timeout=pick_pose_timeout,
        grasp_timeout=grasp_timeout,
        skip_pose=skip_pose,
        capture=capture,
        locate=locate,
        grasp=grasp,
        execute_grasp_motion=execute_grasp_motion,
        results=results,
        product_name=name,
        task_type=task_type,
        snapshot_dir=snapshot_dir,
        pose_results=pose_results,
        capture_results=capture_results,
    )

    summary = {
        "step": 2,
        "run_id": run_id,
        "products": results,
        "jobs": jobs,
        "pending_index": idx,
        "current_product": name,
        "locations": [location],
        "current_location": location,
        "poses": pose_results,
        "captures": capture_results,
    }
    save_plan_state(state_path, summary)
    print(f"      第{idx + 1}件 {name} 抓取结束，下一步跑 plan_3_place.py")
    return summary


def run_step3_place(
    *,
    navigation_url: str,
    pose_url: str,
    nav_timeout: float,
    pose_timeout: float,
    skip_pose: bool,
    nav_target: str | None,
    state_path: Path,
    skip_release: bool = False,
    execute_release_motion: bool = True,
    release_timeout: float = 300.0,
) -> dict[str, Any]:
    """步骤 3：后退点 → 小票/放置点 mark_0 → 放置；末件完成后回 mark_8。"""

    print("=== 步骤 3：后退点 → 小票/放置点 → 放置位姿 → release ===")
    state: dict[str, Any] | None = None
    if state_path.is_file():
        state = load_plan_state(state_path)
    location = resolve_location(nav_target, state)
    if location.upper() == DELIVERY_STATION.upper() and state:
        source = str(state.get("source_location") or "").strip()
        if source:
            location = source.upper()
    jobs = list((state or {}).get("jobs") or [])
    pending_index = int((state or {}).get("pending_index") or 0)
    if jobs and pending_index < len(jobs):
        location = str(jobs[pending_index].get("location") or location).upper()
    run_id = uuid.uuid4().hex[:8]
    nav_results: list[dict[str, Any]] = []
    pose_results: list[dict[str, Any]] = []
    release_results: list[dict[str, Any]] = []

    print(f"[0] GET {navigation_url}/navigation/health")
    wait_nav_ready(navigation_url)
    if not skip_pose:
        print(f"[0] GET {pose_url}/pose/health")
        wait_pose_ready(pose_url)

    navigate_retreat_then_place(
        navigation_url,
        location,
        timeout=nav_timeout,
        run_id=run_id,
        nav_results=nav_results,
        pose_url=pose_url,
        pose_timeout=pose_timeout,
        skip_pose=skip_pose,
        pose_results=pose_results,
        skip_release=skip_release,
        execute_release_motion=execute_release_motion,
        release_timeout=release_timeout,
        release_results=release_results,
    )

    next_index = pending_index + 1
    next_job = jobs[next_index] if next_index < len(jobs) else None
    current_location = DELIVERY_STATION
    if not next_job:
        print(f"      所有商品已放置，返回初始点 {INITIAL_STATION}")
        go_to(
            navigation_url,
            INITIAL_STATION,
            timeout=nav_timeout,
            run_id=run_id,
            role="initial_finish",
            nav_results=nav_results,
            label="初始点",
        )
        current_location = INITIAL_STATION
    summary = {
        "step": 3,
        "run_id": run_id,
        "products": (state or {}).get("products") or [],
        "jobs": jobs,
        "pending_index": next_index,
        "current_product": (next_job or {}).get("product_name"),
        "locations": [location],
        "current_location": current_location,
        "source_location": location,
        "navigation": nav_results,
        "poses": pose_results,
        "releases": release_results,
    }
    save_plan_state(state_path, summary)
    print(f"      已到放置点 {DELIVERY_STATION}，位姿 {POSE_PLACE}，release 完成")
    if next_job:
        print(
            f"      第{pending_index + 1}件已放置。下一件 "
            f"{next_job['product_name']} @ {next_job['location']}"
        )
        print("      第二轮：python3 plan_1_receipt.py --next")
    else:
        print(f"      小票全部完成，已返回初始点 {INITIAL_STATION}")
    return summary


def run(
    *,
    perception_url: str,
    sku_url: str,
    navigation_url: str,
    rgbd_url: str,
    pose_url: str,
    pick_pose_url: str,
    parse_timeout: float,
    sku_timeout: float,
    nav_timeout: float,
    rgbd_timeout: float,
    pose_timeout: float,
    locate_timeout: float,
    pick_pose_timeout: float,
    skip_receipt: bool,
    go_receipt_station: bool,
    navigate: bool,
    capture: bool,
    skip_pose: bool,
    locate: bool,
    grasp: bool,
    execute_grasp_motion: bool,
    nav_target: str | None,
    product_name: str | None,
    task_type: str,
    grasp_timeout: float,
    settle_seconds: float,
    snapshot_dir: Path,
    skip_release: bool = False,
    execute_release_motion: bool = False,
    release_timeout: float = 300.0,
) -> dict[str, Any]:
    run_id = uuid.uuid4().hex[:8]
    nav_results: list[dict[str, Any]] = []
    pose_results: list[dict[str, Any]] = []
    capture_results: list[dict[str, Any]] = []
    release_results: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []

    print(f"[0] GET {navigation_url}/navigation/health")
    wait_nav_ready(navigation_url)
    if not skip_pose:
        print(f"[0] GET {pose_url}/pose/health")
        wait_pose_ready(pose_url)

    if go_receipt_station:
        print(f"[1] 导航到小票位 {RECEIPT_STATION}")
        go_to(
            navigation_url,
            RECEIPT_STATION,
            timeout=nav_timeout,
            run_id=run_id,
            role="receipt",
            nav_results=nav_results,
            label="小票位",
        )
    if not skip_pose and (go_receipt_station or not skip_receipt):
        print(f"[1b] 小票识别位姿 {POSE_RECEIPT}")
        pose_results.append(
            move_to_receipt_pose(
                pose_url,
                timeout=pose_timeout,
                run_id=run_id,
            )
        )
        if settle_seconds > 0:
            print(f"      等待相机稳定 {settle_seconds:.0f}s")
            time.sleep(settle_seconds)

    if skip_receipt:
        print("[2] 跳过小票识别")
        print("[3] 跳过 SKU 查询")
    else:
        print(f"[2] POST {perception_url}/perception/parse")
        product_names = parse_receipt(perception_url, parse_timeout)
        print(f"      product_names = {product_names}")

        print(f"[3] GET {sku_url}/sku/search_by_name × {len(product_names)}")
        results = lookup_products(sku_url, product_names, sku_timeout)

    if navigate:
        jobs = product_jobs(
            results, nav_target=nav_target, product_name=product_name
        )
        print(
            f"      小票 {len(jobs)} 件，逐件抓取放置: "
            + ", ".join(
                f"{job['product_name']}@{job['location']}" for job in jobs
            )
        )
        for round_index, job in enumerate(jobs, start=1):
            name = job["product_name"]
            target_id = job["location"]
            print(
                f"=== 第 {round_index}/{len(jobs)} 件: {name} @ {target_id} ==="
            )
            navigate_to_sku_location(
                navigation_url,
                target_id,
                timeout=nav_timeout,
                run_id=run_id,
                nav_results=nav_results,
                round_index=round_index,
                product_name=name,
            )
            pick_at_location(
                target_id=target_id,
                run_id=run_id,
                perception_url=perception_url,
                rgbd_url=rgbd_url,
                pose_url=pose_url,
                pick_pose_url=pick_pose_url,
                pose_timeout=pose_timeout,
                rgbd_timeout=rgbd_timeout,
                locate_timeout=locate_timeout,
                pick_pose_timeout=pick_pose_timeout,
                grasp_timeout=grasp_timeout,
                skip_pose=skip_pose,
                capture=capture,
                locate=locate,
                grasp=grasp,
                execute_grasp_motion=execute_grasp_motion,
                results=results,
                product_name=name,
                task_type=task_type,
                snapshot_dir=snapshot_dir,
                pose_results=pose_results,
                capture_results=capture_results,
            )
            navigate_retreat_then_place(
                navigation_url,
                target_id,
                timeout=nav_timeout,
                run_id=run_id,
                nav_results=nav_results,
                pose_url=pose_url,
                pose_timeout=pose_timeout,
                skip_pose=skip_pose,
                pose_results=pose_results,
                skip_release=skip_release,
                execute_release_motion=execute_release_motion,
                release_timeout=release_timeout,
                release_results=release_results,
            )

        print(f"=== 所有商品处理完成，返回初始点 {INITIAL_STATION} ===")
        go_to(
            navigation_url,
            INITIAL_STATION,
            timeout=nav_timeout,
            run_id=run_id,
            role="initial_finish",
            nav_results=nav_results,
            label="初始点",
        )

    return {
        "products": results,
        "navigation": nav_results,
        "poses": pose_results,
        "captures": capture_results,
        "releases": release_results,
        "current_location": INITIAL_STATION if navigate else None,
    }


def main() -> int:
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument(
        "--config",
        type=Path,
        default=Path(os.getenv("PLAN_CONFIG", str(DEFAULT_ENDPOINT_CONFIG))),
    )
    bootstrap.add_argument("--profile", default=os.getenv("PLAN_PROFILE", ""))
    bootstrap_args, _ = bootstrap.parse_known_args()
    try:
        active_profile, profile_endpoints = load_endpoint_profile(
            bootstrap_args.config, bootstrap_args.profile
        )
    except RuntimeError as exc:
        print(f"[CONFIG ERROR] {exc}", file=sys.stderr)
        return 2

    parser = argparse.ArgumentParser(
        description="全流程：小票两件则先抓放第一件，再第二轮抓放第二件"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=bootstrap_args.config,
        help="服务端点 YAML，默认 %(default)s",
    )
    parser.add_argument(
        "--profile",
        default=active_profile,
        help="YAML 环境 profile，默认 %(default)s",
    )
    parser.add_argument(
        "--perception-url",
        default=endpoint_default(
            profile_endpoints, "perception_url", "PERCEPTION_URL", DEFAULT_PERCEPTION_URL
        ),
        help="感知服务根地址，默认 %(default)s",
    )
    parser.add_argument(
        "--sku-url",
        default=endpoint_default(
            profile_endpoints, "sku_url", "SKU_API_URL", DEFAULT_SKU_URL
        ),
        help="SKU 服务根地址，默认 %(default)s",
    )
    parser.add_argument(
        "--navigation-url",
        default=endpoint_default(
            profile_endpoints,
            "navigation_url",
            "NAVIGATION_URL",
            DEFAULT_NAVIGATION_URL,
        ),
        help="导航服务根地址，默认 %(default)s",
    )
    parser.add_argument(
        "--rgbd-url",
        default=endpoint_default(
            profile_endpoints, "rgbd_url", "CAMERA_RGBD_URL", DEFAULT_RGBD_URL
        ),
        help="手臂 RGB-D 相机桥，默认 %(default)s",
    )
    parser.add_argument(
        "--pose-url",
        default=endpoint_default(
            profile_endpoints, "pose_url", "POSE_URL", DEFAULT_POSE_URL
        ),
        help="机械臂姿态服务，默认 %(default)s",
    )
    parser.add_argument(
        "--pick-pose-url",
        default=endpoint_default(
            profile_endpoints,
            "pick_pose_url",
            "PICK_POSE_URL",
            DEFAULT_PICK_POSE_URL,
        ),
        help="6D 位姿估计服务，默认 %(default)s",
    )
    parser.add_argument("--parse-timeout", type=float, default=180.0)
    parser.add_argument("--sku-timeout", type=float, default=5.0)
    parser.add_argument("--nav-timeout", type=float, default=600.0)
    parser.add_argument("--rgbd-timeout", type=float, default=30.0)
    parser.add_argument("--pose-timeout", type=float, default=180.0)
    parser.add_argument("--locate-timeout", type=float, default=180.0)
    parser.add_argument("--pick-pose-timeout", type=float, default=180.0)
    parser.add_argument("--grasp-timeout", type=float, default=300.0)
    parser.add_argument("--release-timeout", type=float, default=300.0)
    parser.add_argument(
        "--snapshot-dir",
        type=Path,
        default=DEFAULT_SNAPSHOT_DIR,
        help="RGB-D 保存根目录，默认 %(default)s",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=2.0,
        help="到达小票位后等待相机稳定的秒数",
    )
    parser.add_argument(
        "--skip-receipt-station",
        action="store_true",
        help="已经在小票位时跳过 mark_0",
    )
    parser.add_argument(
        "--nav-target",
        default="",
        help="覆盖货位导航目标；默认用 SKU locations",
    )
    parser.add_argument(
        "--no-navigate",
        action="store_true",
        help="识别后不去货位（仍会先去小票位，除非同时 --skip-receipt-station）",
    )
    parser.add_argument(
        "--skip-receipt",
        action="store_true",
        help="跳过小票/SKU；此时必须给 --nav-target",
    )
    parser.add_argument(
        "--skip-capture",
        action="store_true",
        help="到达货位后不拍手臂 RGB-D",
    )
    parser.add_argument(
        "--skip-pose",
        action="store_true",
        help="不调用机械臂 /pose/prepare",
    )
    parser.add_argument(
        "--skip-locate",
        action="store_true",
        help="拍照后不调用 locate / pick_pose",
    )
    parser.add_argument(
        "--skip-grasp",
        action="store_true",
        help="得到 6D 后不调用 /manipulation/grasp",
    )
    parser.add_argument(
        "--execute-grasp",
        dest="execute_grasp",
        action="store_true",
        default=True,
        help="grasp 时 execute=true，会真实抓取（默认）",
    )
    parser.add_argument(
        "--no-execute-grasp",
        dest="execute_grasp",
        action="store_false",
        help="grasp 时 execute=false，只规划不动臂",
    )
    parser.add_argument(
        "--skip-release",
        action="store_true",
        help="放置位姿后不调用 /manipulation/release",
    )
    parser.add_argument(
        "--execute-release",
        action="store_true",
        help="release 时 execute=true，会真实松爪放置；默认只检查",
    )
    parser.add_argument(
        "--task-type",
        default=DEFAULT_TASK_TYPE,
        choices=["SORTING", "SHORTAGE", "MISPLACED"],
        help="locate 任务类型，默认 %(default)s",
    )
    parser.add_argument(
        "--product-name",
        default="",
        help="覆盖 locate 商品名；默认用 SKU 查询结果",
    )
    args = parser.parse_args()
    print("=== 服务端点配置 ===")
    print(f"配置文件: {args.config}")
    print(f"环境 profile: {args.profile}")
    print(f"perception: {args.perception_url}")
    print(f"sku:        {args.sku_url}")
    print(f"navigation: {args.navigation_url}")
    print(f"rgbd:       {args.rgbd_url}")
    print(f"pose:       {args.pose_url}")
    print(f"pick_pose:  {args.pick_pose_url}")

    nav_target = args.nav_target.strip().upper() or None
    product_name = args.product_name.strip() or None
    if args.skip_receipt and not args.no_navigate and not nav_target:
        parser.error("--skip-receipt 单独导航时需要 --nav-target")

    try:
        summary = run(
            perception_url=args.perception_url.rstrip("/"),
            sku_url=args.sku_url.rstrip("/"),
            navigation_url=args.navigation_url.rstrip("/"),
            rgbd_url=args.rgbd_url.rstrip("/"),
            pose_url=args.pose_url.rstrip("/"),
            pick_pose_url=args.pick_pose_url.rstrip("/"),
            parse_timeout=args.parse_timeout,
            sku_timeout=args.sku_timeout,
            nav_timeout=args.nav_timeout,
            rgbd_timeout=args.rgbd_timeout,
            pose_timeout=args.pose_timeout,
            locate_timeout=args.locate_timeout,
            pick_pose_timeout=args.pick_pose_timeout,
            skip_receipt=args.skip_receipt,
            go_receipt_station=not args.skip_receipt_station,
            navigate=not args.no_navigate,
            capture=not args.skip_capture,
            skip_pose=args.skip_pose,
            locate=not args.skip_locate,
            grasp=not args.skip_grasp,
            execute_grasp_motion=args.execute_grasp,
            nav_target=nav_target,
            product_name=product_name,
            task_type=args.task_type,
            grasp_timeout=args.grasp_timeout,
            settle_seconds=args.settle_seconds,
            snapshot_dir=args.snapshot_dir,
            skip_release=args.skip_release,
            execute_release_motion=args.execute_release,
            release_timeout=args.release_timeout,
        )
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1

    print("\n=== 汇总 ===")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
