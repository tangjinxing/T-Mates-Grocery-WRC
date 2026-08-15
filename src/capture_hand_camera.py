#!/usr/bin/env python3
"""拍头部/左手/右手 D435，保存 RGB + 对齐深度 + camera.json。

优先走 8085 RGB-D 相机桥；连不上再直接打开相机。

  python3 /home/lh/WRC/src/capture_hand_camera.py
  python3 /home/lh/WRC/src/capture_hand_camera.py --camera head
  python3 /home/lh/WRC/src/capture_hand_camera.py --camera left
  python3 /home/lh/WRC/src/capture_hand_camera.py --camera right
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
PARSE_RECEIPT = THIS_DIR / "parse_receipt"
if str(PARSE_RECEIPT) not in sys.path:
    sys.path.insert(0, str(PARSE_RECEIPT))

from rgbd_capture import capture_rgbd

DEFAULT_CAMERA_URL = os.getenv("CAMERA_RGBD_URL", "http://127.0.0.1:8085").rstrip("/")
DEFAULT_OUT_DIR = THIS_DIR / "snapshots"
_NO_PROXY = urllib.request.build_opener(urllib.request.ProxyHandler({}))
CAMERA_CHOICES = ("head", "left", "right", "hands", "all")


def expand_cameras(name: str) -> list[str]:
    if name == "all":
        return ["head", "left", "right"]
    if name == "hands":
        return ["left", "right"]
    return [name]


def capture_via_http(base: str, camera: str, timeout: float) -> dict:
    url = f"{base}/camera/rgbd?camera={camera}&format=json"
    request = urllib.request.Request(url, method="GET")
    try:
        with _NO_PROXY.open(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"GET {url} -> HTTP {exc.code}: {body}") from exc
    except urllib.error.URLError as exc:
        raise ConnectionError(f"GET {url} 连接失败: {exc.reason}") from exc

    payload = json.loads(raw)
    if "rgb_png_b64" not in payload or "depth_png_b64" not in payload:
        raise RuntimeError(f"GET {url} 缺少 rgb/depth 字段")
    return payload


def save_http_payload(payload: dict, out_dir: Path) -> dict[str, Path]:
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
    return {"rgb": rgb_path, "depth": depth_path, "camera": meta_path}


def capture_one(
    camera: str,
    *,
    mode: str,
    camera_url: str,
    timeout: float,
    out_dir: Path,
    width: int,
    height: int,
    fps: int,
    warmup: int,
) -> tuple[dict[str, Path], str]:
    if mode == "direct":
        frame = capture_rgbd(
            camera, width=width, height=height, fps=fps, warmup=warmup
        )
        return frame.save(out_dir), "direct"
    if mode == "http":
        payload = capture_via_http(camera_url, camera, timeout)
        return save_http_payload(payload, out_dir), "http"

    try:
        payload = capture_via_http(camera_url, camera, timeout)
        return save_http_payload(payload, out_dir), "http"
    except ConnectionError:
        print(f"      8085 相机桥不可用，改直接打开 {camera}")
        frame = capture_rgbd(
            camera, width=width, height=height, fps=fps, warmup=warmup
        )
        return frame.save(out_dir), "direct"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="拍头部/手部 RGB-D：rgb.png + depth.png + camera.json"
    )
    parser.add_argument(
        "--camera",
        choices=CAMERA_CHOICES,
        default="all",
        help="head / left / right / hands / all，默认 all",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="保存根目录，默认 %(default)s",
    )
    parser.add_argument(
        "--name",
        default="",
        help="子目录名；多相机时忽略，默认 <camera>_时间戳",
    )
    parser.add_argument(
        "--via",
        choices=["auto", "http", "direct"],
        default="auto",
        help="auto=先 8085 再直连；http=只走 RGB-D 桥；direct=直接打开 D435",
    )
    parser.add_argument(
        "--camera-url",
        default=DEFAULT_CAMERA_URL,
        help="RGB-D HTTP 根地址，默认 %(default)s",
    )
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=20)
    args = parser.parse_args()

    cameras = expand_cameras(args.camera)
    saved: list[Path] = []
    try:
        for camera in cameras:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            folder = args.name if args.name and len(cameras) == 1 else f"{camera}_{stamp}"
            out_dir = args.out_dir / folder
            print(f"[拍] {camera} -> {out_dir}")
            paths, via = capture_one(
                camera,
                mode=args.via,
                camera_url=args.camera_url.rstrip("/"),
                timeout=args.timeout,
                out_dir=out_dir,
                width=args.width,
                height=args.height,
                fps=args.fps,
                warmup=args.warmup,
            )
            saved.extend(paths.values())
            print(
                f"      已保存 ({via}) rgb={paths['rgb'].name} "
                f"depth={paths['depth'].name} meta={paths['camera'].name}"
            )
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1

    print("\n=== 汇总 ===")
    for path in saved:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
