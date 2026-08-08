#!/usr/bin/env python3
"""按配置采集一一对应的 RGB 图像与瑞尔曼末端位姿。"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import yaml


ROOT = Path(__file__).resolve().parent


def load_config(profile_name):
    with (ROOT / "calibration_config.yaml").open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if profile_name not in cfg["profiles"]:
        raise ValueError(f"未知配置 {profile_name}，可选: {', '.join(cfg['profiles'])}")
    profile = cfg["profiles"][profile_name]
    for key in ("arm_ip", "camera_serial"):
        if str(profile.get(key, "")).upper() == "TBD":
            raise ValueError(f"profiles.{profile_name}.{key} 尚未配置")
    board = cfg["charuco"]
    if str(board.get("dictionary", "")).upper() == "TBD" or min(
        int(board.get("squares_x", 0)), int(board.get("squares_y", 0))
    ) < 2 or min(float(board.get("square_length", 0)), float(board.get("marker_length", 0))) <= 0:
        raise ValueError("calibration_config.yaml 中的 ChArUco 参数尚未配置")
    return cfg, profile


def make_detector(board_cfg):
    name = board_cfg["dictionary"]
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"OpenCV 不支持 ArUco 字典 {name}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    board = cv2.aruco.CharucoBoard(
        (int(board_cfg["squares_x"]), int(board_cfg["squares_y"])),
        float(board_cfg["square_length"]),
        float(board_cfg["marker_length"]),
        dictionary,
    )
    return cv2.aruco.CharucoDetector(board)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", help="例如 eye_to_hand_right")
    args = parser.parse_args()
    cfg, profile_cfg = load_config(args.profile)

    sys.path.insert(0, cfg["paths"]["camera_api"])
    sys.path.insert(0, cfg["paths"]["arm_api"])
    from D435_rgb_depth import D435Camera
    from realman_arm_api_api2 import RealmanArmClient

    detector = make_detector(cfg["charuco"])
    data_root = Path(cfg["paths"]["data_root"])
    if not data_root.is_absolute():
        data_root = ROOT / data_root
    scene = data_root / args.profile
    rgb_dir, pose_dir = scene / "rgb", scene / "poses"
    detection_dir, rejected_dir = scene / "detections", scene / "rejected"
    rgb_dir.mkdir(parents=True, exist_ok=True)
    pose_dir.mkdir(parents=True, exist_ok=True)
    detection_dir.mkdir(parents=True, exist_ok=True)
    rejected_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = scene / "samples.json"
    samples = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else []
    write_json(scene / "config_snapshot.json", {"profile": profile_cfg, "charuco": cfg["charuco"], "capture": cfg["capture"]})

    cap = cfg["capture"]
    camera = D435Camera(
        serial=str(profile_cfg["camera_serial"]), width=int(cap["width"]),
        height=int(cap["height"]), fps=int(cap["fps"]),
        enable_color=True, enable_depth=False, align_depth_to_color=False,
    )
    arm = RealmanArmClient(ip=str(profile_cfg["arm_ip"]), model=profile_cfg["arm_side"])
    try:
        arm.connect()
        work_frame = arm.raw_call("rm_get_current_work_frame")
        if any(abs(float(v)) > 1e-9 for v in work_frame.get("pose", [])):
            raise RuntimeError(f"当前工作坐标系不与机械臂基座重合: {work_frame}")
        camera.start()
        for _ in range(int(cap["warmup_frames"])):
            camera.get_frames(timeout_ms=5000)
        intrinsics = camera.get_intrinsics("color")
        intrinsics_name = f"camera_intrinsics_factory_{intrinsics['width']}x{intrinsics['height']}.json"
        write_json(scene / intrinsics_name, intrinsics)
        print(f"当前分辨率内参已保存: {scene / intrinsics_name}")

        print("准备完成：机械臂静止后按 Enter 采集；u 撤回上一组；q 退出。")
        while True:
            command = input("采集> ").strip().lower()
            if command == "q":
                break
            if command == "u":
                if not samples:
                    print("没有可撤回的样本")
                    continue
                last = samples.pop()
                for relative in (last["rgb"], last["pose"], last.get("detection")):
                    if not relative:
                        continue
                    source = scene / relative
                    if source.exists():
                        target = rejected_dir / source.name
                        if target.exists():
                            target = rejected_dir / f"{time.time_ns()}_{source.name}"
                        shutil.move(str(source), str(target))
                write_json(manifest_path, samples)
                print(f"已撤回样本 {int(last['index']):06d}，文件已移入 rejected/")
                continue
            if command:
                print("未知命令：按 Enter 采集，u 撤回，q 退出")
                continue
            host_before_ns = time.time_ns()
            frame = camera.get_frames(timeout_ms=5000)
            state = arm.get_state()
            host_after_ns = time.time_ns()
            errors = [str(v) for v in state.err.get("err", [])]
            if any(v != "0" for v in errors):
                print(f"拒绝采集：机械臂错误 {state.err}")
                continue
            image = frame["color"]
            corners, ids, _, _ = detector.detectBoard(image)
            corner_count = 0 if ids is None else len(ids)
            if corner_count < int(cfg["charuco"]["min_corners"]):
                print(f"拒绝采集：只检测到 {corner_count} 个 ChArUco 角点")
                continue

            index = max([int(s["index"]) for s in samples], default=0) + 1
            stem = f"{index:06d}"
            image_name = stem + str(cap["image_extension"])
            pose_name = stem + ".json"
            if not cv2.imwrite(str(rgb_dir / image_name), image):
                raise RuntimeError("图像保存失败")
            annotated = image.copy()
            if ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(annotated, corners, ids)
            detection_name = stem + ".png"
            if not cv2.imwrite(str(detection_dir / detection_name), annotated):
                raise RuntimeError("检测标注图保存失败")
            pose_record = {
                "index": index, "profile": args.profile,
                "arm_side": profile_cfg["arm_side"], "arm_ip": str(profile_cfg["arm_ip"]),
                "camera_serial": str(profile_cfg["camera_serial"]),
                "camera_timestamp_ms": frame["timestamp_ms"],
                "host_before_ns": host_before_ns, "host_after_ns": host_after_ns,
                "work_frame": work_frame, "joints_deg": state.joints,
                "pose_base_to_gripper_m_rad": state.pose.as_list(), "arm_errors": state.err,
                "charuco_corner_count": corner_count,
            }
            write_json(pose_dir / pose_name, pose_record)
            samples.append({"index": index, "rgb": f"rgb/{image_name}", "pose": f"poses/{pose_name}",
                            "detection": f"detections/{detection_name}"})
            write_json(manifest_path, samples)
            print(f"已保存样本 {stem}，ChArUco 角点 {corner_count}")
    finally:
        camera.stop()
        arm.disconnect()


if __name__ == "__main__":
    main()
