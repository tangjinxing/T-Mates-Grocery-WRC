#!/usr/bin/env python3
"""Five random 20/5 train/validation runs for fixed-board eye-in-hand calibration.

Sample data are read-only. The only output is a JSON report below the scene's
results/ directory. The best run is selected by validation reprojection RMS.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parent


def transform(rotation, translation):
    value = np.eye(4, dtype=float)
    value[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    value[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return value


def pose_to_transform(pose):
    value = np.eye(4, dtype=float)
    value[:3, :3] = Rotation.from_euler("xyz", pose[3:], degrees=False).as_matrix()
    value[:3, 3] = np.asarray(pose[:3], dtype=float)
    return value


def mean_transform(values):
    result = np.eye(4, dtype=float)
    result[:3, :3] = Rotation.from_matrix([value[:3, :3] for value in values]).mean().as_matrix()
    result[:3, 3] = np.mean([value[:3, 3] for value in values], axis=0)
    return result


def metric(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "max": float(np.max(values)),
    }


def make_board(cfg):
    board_cfg = cfg["charuco"]
    name = board_cfg["dictionary"]
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"OpenCV不支持ArUco字典 {name}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    board = cv2.aruco.CharucoBoard(
        (int(board_cfg["squares_x"]), int(board_cfg["squares_y"])),
        float(board_cfg["square_length"]), float(board_cfg["marker_length"]), dictionary,
    )
    return board, cv2.aruco.CharucoDetector(board)


def load_factory_intrinsics(scene, image_size):
    path = scene / f"camera_intrinsics_factory_{image_size[0]}x{image_size[1]}.json"
    if not path.exists():
        raise FileNotFoundError(f"缺少当前分辨率对应的相机内参: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if (int(value["width"]), int(value["height"])) != image_size:
        raise ValueError(f"内参分辨率与图像不一致: {path}")
    matrix = np.array([
        [value["fx"], 0.0, value["ppx"]],
        [0.0, value["fy"], value["ppy"]],
        [0.0, 0.0, 1.0],
    ], dtype=float)
    return path, value, matrix, np.asarray(value["coeffs"], dtype=float)


def load_observations(scene, cfg, expected_profile):
    samples_path = scene / "samples.json"
    if not samples_path.exists():
        raise FileNotFoundError(f"缺少样本清单: {samples_path}")
    samples = json.loads(samples_path.read_text(encoding="utf-8"))
    accepted, seen, problems = [], set(), []
    for item in samples:
        if item.get("status", "accepted") != "accepted":
            continue
        index = int(item["index"])
        if index in seen:
            problems.append(f"重复有效索引{index}")
            continue
        seen.add(index)
        missing = [key for key in ("rgb", "pose") if not (scene / item.get(key, "")).exists()]
        if missing:
            problems.append(f"索引{index}缺少{','.join(missing)}")
            continue
        accepted.append(item)
    if problems:
        print("[WARN] 已只读排除异常清单记录：" + "；".join(problems))

    board, detector = make_board(cfg)
    observations, image_size = [], None
    minimum_corners = int(cfg["charuco"]["min_corners"])
    for item in accepted:
        index = int(item["index"])
        image = cv2.imread(str(scene / item["rgb"]))
        if image is None:
            print(f"[WARN] 只读排除样本{index}: RGB无法读取")
            continue
        current_size = (image.shape[1], image.shape[0])
        if image_size is not None and current_size != image_size:
            raise ValueError("数据集中存在不同图像尺寸")
        image_size = current_size
        corners, ids, _, _ = detector.detectBoard(image)
        count = 0 if ids is None else len(ids)
        if count < minimum_corners:
            print(f"[WARN] 只读排除样本{index}: 原始图ChArUco角点不足({count})")
            continue
        record = json.loads((scene / item["pose"]).read_text(encoding="utf-8"))
        if record.get("profile") != expected_profile:
            raise ValueError(f"样本{index} profile不一致: {record.get('profile')}")
        pose = record.get("pose_base_to_gripper_m_rad")
        if not isinstance(pose, list) or len(pose) != 6 or not np.all(np.isfinite(pose)):
            raise ValueError(f"样本{index}机械臂位姿无效")
        errors = [str(value) for value in record.get("arm_errors", {}).get("err", [])]
        if any(value != "0" for value in errors):
            raise ValueError(f"样本{index}存在机械臂错误: {record.get('arm_errors')}")
        object_points, image_points = board.matchImagePoints(corners, ids)
        observations.append({
            "index": index,
            "base_to_gripper": pose_to_transform(pose),
            "object_points": np.asarray(object_points, dtype=np.float32),
            "image_points": np.asarray(image_points, dtype=np.float32),
            "corner_count": count,
        })
    if image_size is None:
        raise RuntimeError("没有可读取的有效图像")
    return observations, image_size


def solve_pnp(observations, matrix, distortion):
    values = []
    for item in observations:
        ok, rvec, tvec = cv2.solvePnP(
            item["object_points"], item["image_points"], matrix, distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError(f"样本{item['index']} PnP失败")
        rotation, _ = cv2.Rodrigues(rvec)
        values.append({**item, "camera_to_target": transform(rotation, tvec)})
    return values


def calibrate_train(train, method):
    rotation, translation = cv2.calibrateHandEye(
        [item["base_to_gripper"][:3, :3] for item in train],
        [item["base_to_gripper"][:3, 3] for item in train],
        [item["camera_to_target"][:3, :3] for item in train],
        [item["camera_to_target"][:3, 3] for item in train],
        method=method,
    )
    gripper_to_camera = transform(rotation, translation)
    base_to_targets = [
        item["base_to_gripper"] @ gripper_to_camera @ item["camera_to_target"]
        for item in train
    ]
    return gripper_to_camera, mean_transform(base_to_targets)


def evaluate(gripper_to_camera, base_to_target, items, matrix, distortion):
    translations, rotations, reprojections, samples = [], [], [], []
    for item in items:
        predicted_camera_to_target = (
            np.linalg.inv(item["base_to_gripper"] @ gripper_to_camera) @ base_to_target
        )
        delta = np.linalg.inv(item["camera_to_target"]) @ predicted_camera_to_target
        translation_mm = float(np.linalg.norm(delta[:3, 3]) * 1000.0)
        rotation_deg = float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))
        rvec = Rotation.from_matrix(predicted_camera_to_target[:3, :3]).as_rotvec().reshape(3, 1)
        projected, _ = cv2.projectPoints(
            item["object_points"], rvec, predicted_camera_to_target[:3, 3], matrix, distortion,
        )
        difference = projected.reshape(-1, 2) - item["image_points"].reshape(-1, 2)
        reprojection_px = float(np.sqrt(np.mean(np.sum(difference * difference, axis=1))))
        translations.append(translation_mm)
        rotations.append(rotation_deg)
        reprojections.append(reprojection_px)
        samples.append({
            "index": item["index"], "corners": item["corner_count"],
            "translation_mm": translation_mm, "rotation_deg": rotation_deg,
            "reprojection_rms_px": reprojection_px,
        })
    return {
        "translation_mm": metric(translations),
        "rotation_deg": metric(rotations),
        "reprojection_px": metric(reprojections),
        "samples": samples,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "profile",
        choices=["eye_in_hand_left", "eye_in_hand_right", "eye_in_hand_right_v2", "eye_in_hand_right_v3"],
    )
    parser.add_argument("--train", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260805)
    parser.add_argument(
        "--method",
        choices=["TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"],
        help="覆盖calibration_config.yaml中的手眼算法；不同算法写入独立结果文件",
    )
    parser.add_argument(
        "--exclude-indices",
        nargs="*",
        type=int,
        default=[],
        help="只在本次计算中排除指定样本索引；不会修改或删除原始数据",
    )
    args = parser.parse_args()
    cfg = yaml.safe_load((ROOT / "calibration_config.yaml").read_text(encoding="utf-8"))
    profile = cfg["profiles"][args.profile]
    if profile.get("mode") != "eye_in_hand":
        raise ValueError(f"{args.profile}不是眼在手上配置")
    data_root = Path(cfg["paths"]["data_root"])
    scene = (data_root if data_root.is_absolute() else ROOT / data_root) / args.profile
    observations, image_size = load_observations(scene, cfg, args.profile)
    excluded = sorted(set(int(value) for value in args.exclude_indices))
    if excluded:
        available = {int(item["index"]) for item in observations}
        unknown = [index for index in excluded if index not in available]
        if unknown:
            raise ValueError(f"请求排除的样本索引不存在或原本已无效: {unknown}")
        observations = [item for item in observations if int(item["index"]) not in excluded]
        print(f"[INFO] 本次只读排除样本索引={excluded}；原始文件和samples.json保持不变")
    if len(observations) <= args.train:
        raise RuntimeError(f"独立有效样本只有{len(observations)}组，必须多于训练数量{args.train}")
    intrinsics_path, intrinsics, matrix, distortion = load_factory_intrinsics(scene, image_size)
    observations = solve_pnp(observations, matrix, distortion)
    print(f"[INFO] profile={args.profile}，独立有效样本={len(observations)}，图像={image_size[0]}x{image_size[1]}")
    print(f"[INFO] 内参={intrinsics_path}，每轮={args.train}训练/{len(observations)-args.train}验证，重复{args.repeats}次")

    method_name = str(args.method or cfg["calibration"]["hand_eye_method"]).upper()
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD, "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    if method_name not in methods:
        raise ValueError(f"不支持手眼算法{method_name}")
    runs = []
    for repeat in range(args.repeats):
        rng = np.random.default_rng(args.seed + repeat)
        order = rng.permutation(len(observations))
        train = [observations[index] for index in order[:args.train]]
        validation = [observations[index] for index in order[args.train:]]
        gripper_to_camera, base_to_target = calibrate_train(train, methods[method_name])
        training_report = evaluate(gripper_to_camera, base_to_target, train, matrix, distortion)
        validation_report = evaluate(gripper_to_camera, base_to_target, validation, matrix, distortion)
        run = {
            "repeat": repeat + 1, "seed": args.seed + repeat,
            "training_indices": sorted(item["index"] for item in train),
            "validation_indices": sorted(item["index"] for item in validation),
            "gripper_T_camera": gripper_to_camera.tolist(),
            "base_T_charuco": base_to_target.tolist(),
            "training": training_report, "validation": validation_report,
        }
        runs.append(run)
        print(
            f"repeat {repeat+1}: validation reprojection RMS={validation_report['reprojection_px']['rms']:.4f}px, "
            f"translation RMS={validation_report['translation_mm']['rms']:.3f}mm, "
            f"rotation RMS={validation_report['rotation_deg']['rms']:.4f}deg"
        )
    best_index = min(range(len(runs)), key=lambda index: runs[index]["validation"]["reprojection_px"]["rms"])
    best = runs[best_index]
    output = {
        "profile": args.profile, "mode": "eye_in_hand",
        "model": "fixed gripper_T_camera with externally fixed ChArUco board",
        "split": f"random_{args.train}_train_{len(observations)-args.train}_validation",
        "repeats": args.repeats, "selection_metric": "minimum validation reprojection RMS px",
        "hand_eye_method": method_name, "intrinsic_source": "realsense_factory",
        "intrinsics_file": str(intrinsics_path.relative_to(ROOT.parent)),
        "camera_matrix": matrix.tolist(), "distortion": distortion.reshape(-1).tolist(),
        "excluded_indices": excluded,
        "valid_sample_count": len(observations), "best_repeat": best_index + 1,
        "best_gripper_T_camera": best["gripper_T_camera"],
        "best_validation": best["validation"], "runs": runs,
    }
    method_suffix = "" if args.method is None else "_" + method_name.lower()
    exclusion_suffix = "" if not excluded else "_exclude_" + "-".join(str(value) for value in excluded)
    result_path = scene / "results" / (
        f"eye_in_hand_{args.train}_{len(observations)-args.train}"
        f"{method_suffix}{exclusion_suffix}_five_repeats.json"
    )
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print("\n========== 最优结果 ==========")
    print(f"best repeat: {best_index+1}")
    print("gripper_T_camera:")
    print(np.array2string(np.asarray(best["gripper_T_camera"]), precision=10, suppress_small=False))
    report = best["validation"]
    print(f"validation reprojection RMS={report['reprojection_px']['rms']:.4f}px")
    print(f"validation translation RMS={report['translation_mm']['rms']:.4f}mm")
    print(f"validation rotation RMS={report['rotation_deg']['rms']:.4f}deg")
    print(f"结果已保存: {result_path}")


if __name__ == "__main__":
    main()
