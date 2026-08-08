#!/usr/bin/env python3
"""从统一 YAML 读取拍照预设，自动到位后采集对齐的 D435 RGB/深度图。"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
PRESET_PATH = ROOT / "config" / "preset_poses.yaml"
CONSOLE_ROOT = ROOT.parent / "initial_pose_console"
ROBOT_CONFIG_PATH = CONSOLE_ROOT / "control_config.yaml"
CAMERA_API_ROOT = Path("/home/lh/robot_api/camera_api")

CAMERAS = {
    "head": "344422070170",
    "left": "215322079194",
    "right": "335522072306",
}

MODES = {
    "eye_to_hand_left": {"camera": "head"},
    "eye_to_hand_right": {"camera": "head"},
    "eye_in_hand_left": {"camera": "left"},
    "eye_in_hand_right": {"camera": "right"},
}

COMPONENT_ORDER = ("torso", "head", "left_arm", "right_arm")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取拍照姿态并采集一组1280x720 RGB和对齐深度图",
    )
    parser.add_argument("--mode", required=True, choices=sorted(MODES))
    parser.add_argument(
        "--shelf-level",
        choices=("1", "2", "3"),
        default="2",
        help="货架层级1/2/3；默认2",
    )
    parser.add_argument(
        "--preset",
        help="精确指定统一YAML预设名；提供后覆盖--shelf-level自动生成的名称",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=30, help="保存前丢弃的预热帧数")
    parser.add_argument("--arm-speed", type=float, default=5.0, help="机械臂 MoveJ_P 速度百分比")
    parser.add_argument("--lift-speed", type=int, default=5, help="升降柱速度百分比")
    return parser.parse_args()


def resolve_preset_name(mode: str, shelf_level: str, override: str | None) -> str:
    if override:
        return override
    if mode.startswith("eye_in_hand_"):
        arm = mode.removeprefix("eye_in_hand_")
        return f"level_{shelf_level}_{arm}"
    raise ValueError("眼在手外模式必须通过--preset明确指定预设")


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"配置文件不存在: {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML顶层不是对象: {path}")
    return data


def load_preset(name: str) -> dict[str, Any]:
    registry = load_yaml(PRESET_PATH)
    presets = registry.get("presets")
    if not isinstance(presets, dict) or name not in presets:
        raise ValueError(f"统一 YAML 中尚未创建预设: {name}")
    preset = presets[name]
    if not isinstance(preset, dict):
        raise ValueError(f"预设结构无效: {name}")
    return preset


def validate_preset(preset_name: str, preset: dict[str, Any]) -> list[str]:
    applied = preset.get("apply_components")
    if not isinstance(applied, list) or not applied:
        raise ValueError(f"预设 {preset_name} 没有有效的 apply_components")
    unknown = [item for item in applied if item not in COMPONENT_ORDER]
    if unknown:
        raise ValueError(f"预设包含未知执行部件: {unknown}")

    if "torso" in applied:
        height = preset.get("torso", {}).get("height")
        if height is None or not 100 <= int(height) <= 1350:
            raise ValueError(f"参与执行的躯干高度无效: {height!r}")
    if "head" in applied:
        head = preset.get("head")
        if not isinstance(head, dict) or head.get("yaw") is None or head.get("pitch") is None:
            raise ValueError("参与执行的头部缺少 yaw/pitch")
    for component in ("left_arm", "right_arm"):
        if component not in applied:
            continue
        values = preset.get(component, {}).get("pose_6d")
        if not isinstance(values, list) or len(values) != 6:
            raise ValueError(f"参与执行的 {component} 缺少长度为6的 pose_6d")
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{component}.pose_6d 包含非法数值")
    return [component for component in COMPONENT_ORDER if component in applied]


def show_expected_pose(mode: str, preset_name: str, preset: dict[str, Any], applied: list[str]) -> None:
    print("\n========== 本次采集 ==========")
    print(f"模式: {mode}")
    print(f"统一预设: {preset_name}")
    print(f"读取文件: {PRESET_PATH}")
    print(f"只执行: {', '.join(applied)}")
    if "torso" in applied:
        print(f"躯干目标高度: {int(preset['torso']['height'])} mm")
    if "head" in applied:
        print(f"头部目标: yaw={preset['head']['yaw']}, pitch={preset['head']['pitch']}")
    for component in ("left_arm", "right_arm"):
        if component in applied:
            values = preset[component]["pose_6d"]
            print(f"{component} 目标6D [{preset[component].get('frame')}]:")
            print("  " + ", ".join(f"{float(value):.6f}" for value in values))
    skipped = [component for component in COMPONENT_ORDER if component not in applied]
    print(f"保持不动: {', '.join(skipped) if skipped else '无'}")


def move_to_preset(preset: dict[str, Any], applied: list[str], arm_speed: float, lift_speed: int):
    config = load_yaml(ROBOT_CONFIG_PATH)
    sys.path.insert(0, str(CONSOLE_ROOT))
    from hardware.robot_manager import RobotManager

    robot = RobotManager(config)
    try:
        # 躯干状态和控制均通过左臂控制器；其他部件只按 apply_components 连接。
        required_arms = set()
        if "torso" in applied or "left_arm" in applied:
            required_arms.add("left")
        if "right_arm" in applied:
            required_arms.add("right")
        for side in ("left", "right"):
            if side in required_arms:
                print(f"连接{side}臂控制器...")
                robot.connect_arm(side)
        if "head" in applied:
            print("连接头部控制器...")
            robot.connect_head()

        if "torso" in applied:
            target = int(preset["torso"]["height"])
            actual = robot.get_actual_lift_height()
            if abs(actual - target) > int(config["lift"].get("arrival_tolerance", 10)):
                print(f"移动躯干: {actual} -> {target} mm")
                robot.move_lift(target, lift_speed, confirmed=True)
            else:
                print(f"躯干已到位: {actual} mm")
        if "head" in applied:
            head = preset["head"]
            print(f"移动头部: yaw={head['yaw']}, pitch={head['pitch']}")
            robot.move_head(int(head["yaw"]), int(head["pitch"]), confirmed=True)
        for side, component in (("left", "left_arm"), ("right", "right_arm")):
            if component in applied:
                target = [float(value) for value in preset[component]["pose_6d"]]
                print(f"低速移动{side}臂到拍照位姿，速度={arm_speed}%")
                robot.move_arm_pose(side, target, arm_speed, confirmed=True)
        print("预设中的全部执行部件均已完成到位。")
        return robot
    except Exception:
        robot.close()
        raise


def save_json(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def validate_sample_directory(path: Path) -> None:
    """发布前确认三项数据完整且彼此尺寸一致。"""
    required = [path / "rgb.png", path / "depth.png", path / "camera.json"]
    missing = [item.name for item in required if not item.is_file()]
    if missing:
        raise RuntimeError(f"临时样本缺少文件: {missing}")
    rgb = cv2.imread(str(path / "rgb.png"), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(path / "depth.png"), cv2.IMREAD_UNCHANGED)
    if rgb is None or depth is None:
        raise RuntimeError("临时样本RGB或深度图无法重新读取")
    if rgb.shape[:2] != depth.shape[:2] or depth.dtype != np.uint16:
        raise RuntimeError(
            f"临时样本图像不一致: RGB={rgb.shape}, depth={depth.shape}/{depth.dtype}"
        )
    value = json.loads((path / "camera.json").read_text(encoding="utf-8"))
    if not isinstance(value.get("cam_K"), list) or len(value["cam_K"]) != 9:
        raise RuntimeError("临时样本camera.json缺少长度为9的cam_K")
    if float(value.get("depth_scale", 0.0)) <= 0.0:
        raise RuntimeError("临时样本camera.json的depth_scale无效")


def publish_latest_sample(mode_root: Path, temporary: Path) -> tuple[Path, Path]:
    """将完整临时样本轮换为latest，并最多保留一组previous。"""
    latest = mode_root / "latest_sample"
    previous = mode_root / "previous_sample"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    old_previous = mode_root / f".old_previous_{stamp}"

    # 允许预先创建的空占位目录；任何含数据但不完整的目录都拒绝覆盖。
    for target in (latest, previous):
        if target.exists() and not any(target.iterdir()):
            target.rmdir()
        elif target.exists():
            validate_sample_directory(target)

    previous_stashed = False
    latest_moved = False
    new_published = False
    try:
        if previous.exists():
            previous.rename(old_previous)
            previous_stashed = True
        if latest.exists():
            latest.rename(previous)
            latest_moved = True
        temporary.rename(latest)
        new_published = True
    except Exception:
        # 尽最大可能恢复操作前状态，避免一次失败同时丢失两代样本。
        if new_published and latest.exists():
            latest.rename(temporary)
        if latest_moved and previous.exists():
            previous.rename(latest)
        if previous_stashed and old_previous.exists():
            old_previous.rename(previous)
        raise
    if old_previous.exists():
        shutil.rmtree(old_previous)
    return latest, previous


def main() -> int:
    args = parse_args()
    mode_config = MODES[args.mode]
    preset_name = resolve_preset_name(args.mode, args.shelf_level, args.preset)
    preset = load_preset(preset_name)
    applied = validate_preset(preset_name, preset)
    show_expected_pose(args.mode, preset_name, preset, applied)

    confirmation = f"MOVE {preset_name}"
    answer = input(f"\n确认周围无人与障碍物，输入 {confirmation} 自动到位并采集: ").strip()
    if answer != confirmation:
        print("已取消，没有连接硬件、移动机器人或创建样本。")
        return 0

    robot = move_to_preset(preset, applied, args.arm_speed, args.lift_speed)
    camera = None
    temporary_sample_dir: Path | None = None
    try:
        sys.path.insert(0, str(CAMERA_API_ROOT))
        from D435_rgb_depth import D435Camera

        camera_name = mode_config["camera"]
        serial = CAMERAS[camera_name]
        camera = D435Camera(
            serial=serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            enable_color=True,
            enable_depth=True,
            align_depth_to_color=True,
        )
        camera.start()
        packet = None
        for index in range(args.warmup + 1):
            packet = camera.get_frames(timeout_ms=5000)
            if index < args.warmup:
                continue

        if packet is None or packet.get("color") is None or packet.get("depth") is None:
            raise RuntimeError("没有同时取得RGB和深度图")

        color = packet["color"]
        depth = packet["depth"]
        if color.shape[:2] != depth.shape[:2]:
            raise RuntimeError(f"RGB和对齐深度尺寸不一致: {color.shape} / {depth.shape}")
        if depth.dtype != np.uint16:
            raise RuntimeError(f"深度图类型不是预期的 uint16: {depth.dtype}")

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        mode_root = ROOT / "runs" / args.mode
        mode_root.mkdir(parents=True, exist_ok=True)
        sample_dir = mode_root / f".tmp_sample_{stamp}"
        sample_dir.mkdir(parents=False, exist_ok=False)
        temporary_sample_dir = sample_dir
        rgb_path = sample_dir / "rgb.png"
        depth_path = sample_dir / "depth.png"

        if not cv2.imwrite(str(rgb_path), color):
            raise RuntimeError(f"RGB保存失败: {rgb_path}")
        if not cv2.imwrite(str(depth_path), depth):
            raise RuntimeError(f"深度图保存失败: {depth_path}")

        intrinsics = camera.get_intrinsics("color")
        depth_scale_m = float(packet["depth_scale"])
        camera_json = {
            "cam_K": [
                float(intrinsics["fx"]), 0.0, float(intrinsics["ppx"]),
                0.0, float(intrinsics["fy"]), float(intrinsics["ppy"]),
                0.0, 0.0, 1.0,
            ],
            # 网页采用BOP风格：depth_mm = depth_raw * depth_scale。
            "depth_scale": round(depth_scale_m * 1000.0, 6),
            "mode": args.mode,
            "preset": preset_name,
            "camera_serial": serial,
            "captured_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "image_width": int(color.shape[1]),
            "image_height": int(color.shape[0]),
        }
        save_json(sample_dir / "camera.json", camera_json)

        validate_sample_directory(sample_dir)
        latest_dir, previous_dir = publish_latest_sample(mode_root, sample_dir)
        rgb_path = latest_dir / "rgb.png"
        depth_path = latest_dir / "depth.png"

        print("\n采集成功:")
        print(f"  RGB:   {rgb_path}")
        print(f"  Depth: {depth_path} (uint16原始深度)")
        print(f"  Camera:{latest_dir / 'camera.json'}")
        print(f"  Latest: {latest_dir}")
        print(f"  Previous: {previous_dir if previous_dir.exists() else '无（首次采集）'}")
        print(f"  depth_scale: {camera_json['depth_scale']} mm/unit")
        return 0
    finally:
        if camera is not None:
            camera.stop()
        if temporary_sample_dir is not None and temporary_sample_dir.exists():
            shutil.rmtree(temporary_sample_dir)
        robot.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n用户中止。")
        raise SystemExit(130)
    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        print("如果提示 Device or resource busy，请先在8010页面点击“释放相机”。", file=sys.stderr)
        raise SystemExit(1)
