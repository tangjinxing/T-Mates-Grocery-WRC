#!/usr/bin/env python3
"""Five stratified 160/remaining evaluations for moving-head eye-to-hand calibration.

This program is read-only with respect to samples: it only writes new reports below results/.
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from dynamic_handeye_web.dynamic_calibration import make_board, pose_to_transform, transform

ROOT = Path(__file__).resolve().parent


def pose6_to_transform(value):
    output = np.eye(4)
    output[:3, :3] = Rotation.from_rotvec(value[:3]).as_matrix()
    output[:3, 3] = value[3:]
    return output


def transform_to_pose6(value):
    return np.r_[Rotation.from_matrix(value[:3, :3]).as_rotvec(), value[:3, 3]]


def twist_exp(value):
    """SE(3) exponential; value is [rotation-vector, translational twist]."""
    omega, velocity = np.asarray(value[:3]), np.asarray(value[3:])
    theta = np.linalg.norm(omega)
    output = np.eye(4)
    output[:3, :3] = Rotation.from_rotvec(omega).as_matrix()
    skew = np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])
    if theta < 1e-10:
        jacobian = np.eye(3) + 0.5 * skew
    else:
        jacobian = np.eye(3) + (1 - np.cos(theta)) / theta**2 * skew + (theta - np.sin(theta)) / theta**3 * (skew @ skew)
    output[:3, 3] = jacobian @ velocity
    return output


def camera_model(parameters, yaw, pitch):
    # Reference values improve numerical conditioning and fix the model gauge.
    dy, dp = (float(yaw) - 500.0) / 100.0, (float(pitch) - 450.0) / 50.0
    return pose6_to_transform(parameters[:6]) @ twist_exp(parameters[6:12] * dy) @ twist_exp(parameters[12:18] * dp)


def delta_residual(reference, estimate):
    delta = np.linalg.inv(reference) @ estimate
    return np.r_[delta[:3, 3] * 1000.0, Rotation.from_matrix(delta[:3, :3]).as_rotvec() * 100.0]


def mean_transform(values):
    output = np.eye(4)
    output[:3, 3] = np.mean([x[:3, 3] for x in values], axis=0)
    output[:3, :3] = Rotation.from_matrix([x[:3, :3] for x in values]).mean().as_matrix()
    return output


def load_observations(scene, cfg):
    samples = json.loads((scene / "samples.json").read_text(encoding="utf-8"))
    accepted, seen, problems = [], set(), []
    for item in samples:
        if item.get("status", "accepted") != "accepted":
            continue
        index = int(item["index"])
        if index in seen:
            problems.append(f"重复有效索引 {index}")
            continue
        seen.add(index)
        missing = [key for key in ("rgb", "pose") if not (scene / item[key]).exists()]
        if missing:
            problems.append(f"索引 {index} 缺少 {','.join(missing)}")
            continue
        accepted.append(item)
    grouped = defaultdict(list)
    for item in accepted:
        grouped[item["node_id"]].append(item)
    expected_nodes = [x["id"] for x in cfg["dynamic_eye_to_hand"]["nodes"]]
    counts = {node: len(grouped[node]) for node in expected_nodes}
    if any(value < 16 for value in counts.values()):
        raise RuntimeError(f"数据不足：独立有效样本={len(accepted)}，各节点={counts}；每个节点至少需要16组训练数据。")
    if problems:
        print("[WARN] 清单异常记录已只读排除：" + "；".join(problems))
    print(f"[INFO] 独立有效样本={len(accepted)}，各节点={counts}")

    board, detector = make_board(cfg)
    intrinsics_path = scene / "camera_intrinsics_factory_1280x720.json"
    intr = json.loads(intrinsics_path.read_text(encoding="utf-8"))
    matrix = np.array([[intr["fx"], 0, intr["ppx"]], [0, intr["fy"], intr["ppy"]], [0, 0, 1]], float)
    distortion = np.asarray(intr["coeffs"], float)
    observations = []
    for item in accepted:
        image = cv2.imread(str(scene / item["rgb"]))
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is None or len(ids) < int(cfg["charuco"]["min_corners"]):
            raise RuntimeError(f"样本{item['index']}角点不足")
        object_points, image_points = board.matchImagePoints(corners, ids)
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, matrix, distortion)
        if not ok:
            raise RuntimeError(f"样本{item['index']} PnP失败")
        rotation, _ = cv2.Rodrigues(rvec)
        record = json.loads((scene / item["pose"]).read_text(encoding="utf-8"))
        observations.append({
            "index": int(item["index"]), "node_id": item["node_id"],
            "yaw": float(record["head_actual"]["yaw"]), "pitch": float(record["head_actual"]["pitch"]),
            "base_to_gripper": pose_to_transform(record["pose_base_to_gripper_m_rad"]),
            "camera_to_board": transform(rotation, tvec),
            "object_points": object_points, "image_points": image_points,
        })
    return observations, board, matrix, distortion


def node_handeye(items):
    robot = [np.linalg.inv(x["base_to_gripper"]) for x in items]
    camera = [x["camera_to_board"] for x in items]
    rotation, translation = cv2.calibrateHandEye(
        [x[:3, :3] for x in robot], [x[:3, 3] for x in robot],
        [x[:3, :3] for x in camera], [x[:3, 3] for x in camera],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    return transform(rotation, translation)


def initialize(train):
    grouped = defaultdict(list)
    for item in train:
        grouped[item["node_id"]].append(item)
    node_poses = []
    for items in grouped.values():
        pose = node_handeye(items)
        node_poses.append((np.mean([x["yaw"] for x in items]), np.mean([x["pitch"] for x in items]), pose))
    reference = min(node_poses, key=lambda x: abs(x[0] - 500) + abs(x[1] - 450))[2]
    initial_model = np.r_[transform_to_pose6(reference), np.zeros(12)]

    def node_cost(value):
        return np.concatenate([delta_residual(pose, camera_model(value, yaw, pitch)) for yaw, pitch, pose in node_poses])
    model = least_squares(node_cost, initial_model, loss="soft_l1", f_scale=2.0, max_nfev=1500).x
    board_values = [np.linalg.inv(x["base_to_gripper"]) @ camera_model(model, x["yaw"], x["pitch"]) @ x["camera_to_board"] for x in train]
    return np.r_[model, transform_to_pose6(mean_transform(board_values))]


def fit(train):
    initial = initialize(train)
    def cost(value):
        gripper_to_board = pose6_to_transform(value[18:24])
        residuals = []
        for item in train:
            predicted = np.linalg.inv(camera_model(value, item["yaw"], item["pitch"])) @ item["base_to_gripper"] @ gripper_to_board
            residuals.append(delta_residual(item["camera_to_board"], predicted))
        return np.concatenate(residuals)
    result = least_squares(cost, initial, loss="soft_l1", f_scale=2.0, max_nfev=2500)
    if not result.success:
        raise RuntimeError(f"全局优化未收敛: {result.message}")
    return result.x, float(np.sqrt(np.mean(cost(result.x) ** 2))), int(result.nfev)


def evaluate(parameters, items, matrix, distortion):
    gripper_to_board = pose6_to_transform(parameters[18:24])
    translation, rotation, pixels, per_sample = [], [], [], []
    for item in items:
        predicted = np.linalg.inv(camera_model(parameters, item["yaw"], item["pitch"])) @ item["base_to_gripper"] @ gripper_to_board
        delta = np.linalg.inv(item["camera_to_board"]) @ predicted
        t_mm = float(np.linalg.norm(delta[:3, 3]) * 1000)
        r_deg = float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))
        rvec = Rotation.from_matrix(predicted[:3, :3]).as_rotvec().reshape(3, 1)
        projected, _ = cv2.projectPoints(item["object_points"], rvec, predicted[:3, 3], matrix, distortion)
        diff = projected.reshape(-1, 2) - item["image_points"].reshape(-1, 2)
        px = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))
        translation.append(t_mm); rotation.append(r_deg); pixels.append(px)
        per_sample.append({"index": item["index"], "node_id": item["node_id"], "translation_mm": t_mm, "rotation_deg": r_deg, "reprojection_rms_px": px})
    metric = lambda values: {"mean": float(np.mean(values)), "rms": float(np.sqrt(np.mean(np.square(values)))), "max": float(np.max(values))}
    return {"translation_mm": metric(translation), "rotation_deg": metric(rotation), "reprojection_px": metric(pixels), "samples": per_sample}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="../datasets/dynamic_eye_to_hand_right")
    parser.add_argument("--seed", type=int, default=20260804)
    args = parser.parse_args()
    cfg = yaml.safe_load((ROOT / "calibration_config.yaml").read_text(encoding="utf-8"))
    scene = Path(args.scene); scene = scene if scene.is_absolute() else ROOT / scene
    observations, _, matrix, distortion = load_observations(scene, cfg)
    grouped = defaultdict(list)
    for item in observations: grouped[item["node_id"]].append(item)
    runs = []
    for repeat in range(5):
        rng = np.random.default_rng(args.seed + repeat)
        train, validation = [], []
        for node_id in sorted(grouped):
            indices = rng.permutation(len(grouped[node_id]))
            train.extend(grouped[node_id][i] for i in indices[:16])
            validation.extend(grouped[node_id][i] for i in indices[16:])
        parameters, training_cost, nfev = fit(train)
        report = evaluate(parameters, validation, matrix, distortion)
        runs.append({
            "repeat": repeat + 1, "seed": args.seed + repeat,
            "training_indices": sorted(x["index"] for x in train),
            "validation_indices": sorted(x["index"] for x in validation),
            "training_count": len(train), "validation_count": len(validation),
            "optimizer_pose_cost_rms": training_cost, "optimizer_evaluations": nfev,
            "parameters": parameters.tolist(), "validation": report,
        })
        print(f"repeat {repeat+1}: validation reprojection RMS={report['reprojection_px']['rms']:.4f}px, translation RMS={report['translation_mm']['rms']:.3f}mm, rotation RMS={report['rotation_deg']['rms']:.4f}deg")
    best = min(range(5), key=lambda i: runs[i]["validation"]["reprojection_px"]["rms"])
    output = {
        "model": "base_T_camera(yaw,pitch)=T0*Exp(xi_y*dy)*Exp(xi_p*dp)",
        "split": "stratified_per_node_16_train_all_remaining_validation", "repeats": 5,
        "best_repeat": best + 1, "best_parameters": runs[best]["parameters"], "runs": runs,
    }
    result_path = scene / "results" / f"dynamic_160_{len(runs[0]['validation_indices'])}_five_repeats.json"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"结果已保存: {result_path}")


if __name__ == "__main__":
    main()
