#!/usr/bin/env python3
"""用 ChArUco 数据计算 eye-in-hand 或 eye-to-hand 手眼矩阵。"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parent


def transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation).reshape(3, 3)
    result[:3, 3] = np.asarray(translation).reshape(3)
    return result


def pose_to_transform(pose):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", pose[3:], degrees=False).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def board_and_detector(cfg):
    board_cfg = cfg["charuco"]
    name = board_cfg["dictionary"]
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"OpenCV 不支持 ArUco 字典 {name}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    board = cv2.aruco.CharucoBoard(
        (int(board_cfg["squares_x"]), int(board_cfg["squares_y"])),
        float(board_cfg["square_length"]), float(board_cfg["marker_length"]), dictionary,
    )
    return board, cv2.aruco.CharucoDetector(board)


def residuals(mode, base_to_gripper, camera_to_target, camera_to_output):
    constants = []
    for b_t_g, c_t_t in zip(base_to_gripper, camera_to_target):
        if mode == "eye_in_hand":
            constants.append(b_t_g @ camera_to_output @ c_t_t)  # base_T_target
        else:
            constants.append(np.linalg.inv(b_t_g) @ camera_to_output @ c_t_t)  # gripper_T_target
    reference = constants[0]
    translation_mm, rotation_deg = [], []
    for value in constants:
        delta = np.linalg.inv(reference) @ value
        translation_mm.append(float(np.linalg.norm(delta[:3, 3]) * 1000.0))
        rotation_deg.append(float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())))
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(np.square(translation_mm)))),
        "translation_max_mm": max(translation_mm),
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotation_deg)))),
        "rotation_max_deg": max(rotation_deg),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", help="例如 eye_to_hand_right")
    args = parser.parse_args()
    with (ROOT / "calibration_config.yaml").open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    profile = cfg["profiles"].get(args.profile)
    if profile is None:
        raise ValueError(f"未知配置 {args.profile}")
    data_root = Path(cfg["paths"]["data_root"])
    scene = (data_root if data_root.is_absolute() else ROOT / data_root) / args.profile
    samples = json.loads((scene / "samples.json").read_text(encoding="utf-8"))
    board, detector = board_and_detector(cfg)

    detections, records, image_size = [], [], None
    for sample in samples:
        image = cv2.imread(str(scene / sample["rgb"]))
        if image is None:
            print(f"跳过 {sample['index']}: 图像无法读取")
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is not None and size != image_size:
            raise ValueError("数据集中存在不同图像尺寸")
        image_size = size
        corners, ids, _, _ = detector.detectBoard(image)
        count = 0 if ids is None else len(ids)
        if count < int(cfg["charuco"]["min_corners"]):
            print(f"跳过 {sample['index']}: ChArUco 角点不足 ({count})")
            continue
        detections.append((corners, ids))
        records.append(json.loads((scene / sample["pose"]).read_text(encoding="utf-8")))

    minimum = int(cfg["calibration"]["min_samples"])
    if len(records) < minimum:
        raise RuntimeError(f"有效样本只有 {len(records)}，至少需要 {minimum}")

    if cfg["calibration"].get("estimate_intrinsics", True):
        reprojection, camera_matrix, distortion, _, _ = cv2.aruco.calibrateCameraCharuco(
            [v[0] for v in detections], [v[1] for v in detections], board,
            image_size, None, None,
        )
        intrinsic_source = "charuco_dataset"
    else:
        intrinsics_path = scene / f"camera_intrinsics_factory_{image_size[0]}x{image_size[1]}.json"
        if not intrinsics_path.exists():
            raise FileNotFoundError(f"缺少当前图像分辨率对应的内参文件: {intrinsics_path}")
        factory = json.loads(intrinsics_path.read_text(encoding="utf-8"))
        camera_matrix = np.array([[factory["fx"], 0, factory["ppx"]],
                                  [0, factory["fy"], factory["ppy"]], [0, 0, 1]], dtype=float)
        distortion = np.asarray(factory["coeffs"], dtype=float)
        reprojection, intrinsic_source = None, "realsense_factory"

    camera_to_target, base_to_gripper = [], []
    used = []
    for detection, record in zip(detections, records):
        object_points, image_points = board.matchImagePoints(*detection)
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distortion)
        if not ok:
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        camera_to_target.append(transform(rotation, tvec))
        base_to_gripper.append(pose_to_transform(record["pose_base_to_gripper_m_rad"]))
        used.append(record["index"])
    if len(used) < minimum:
        raise RuntimeError(f"PnP 成功样本只有 {len(used)}，至少需要 {minimum}")

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method_name = str(cfg["calibration"]["hand_eye_method"]).upper()
    if profile["mode"] == "eye_in_hand":
        robot_inputs = base_to_gripper
        output_name = "gripper_to_camera"
    else:
        robot_inputs = [np.linalg.inv(value) for value in base_to_gripper]
        output_name = "base_to_camera"
    rotation, translation = cv2.calibrateHandEye(
        [v[:3, :3] for v in robot_inputs], [v[:3, 3] for v in robot_inputs],
        [v[:3, :3] for v in camera_to_target], [v[:3, 3] for v in camera_to_target],
        method=methods[method_name],
    )
    output_transform = transform(rotation, translation)
    result = {
        "profile": args.profile, "mode": profile["mode"], "sample_indices": used,
        "intrinsic_source": intrinsic_source,
        "reprojection_rms_px": None if reprojection is None else float(reprojection),
        "camera_matrix": camera_matrix.tolist(), "distortion": distortion.reshape(-1).tolist(),
        "hand_eye_method": method_name, "output_transform_name": output_name,
        "transform": output_transform.tolist(),
        "consistency": residuals(profile["mode"], base_to_gripper, camera_to_target, output_transform),
    }
    output = scene / "calibration_result.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"结果已保存: {output}")


if __name__ == "__main__":
    main()
