#!/usr/bin/env python3
"""Moving-head eye-to-hand calibration: solve one base_T_camera per head node."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def transform(rotation, translation):
    value = np.eye(4, dtype=float)
    value[:3, :3] = np.asarray(rotation).reshape(3, 3)
    value[:3, 3] = np.asarray(translation).reshape(3)
    return value


def pose_to_transform(pose):
    value = np.eye(4, dtype=float)
    value[:3, :3] = Rotation.from_euler("xyz", pose[3:], degrees=False).as_matrix()
    value[:3, 3] = pose[:3]
    return value


def make_board(cfg):
    board_cfg = cfg["charuco"]
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, board_cfg["dictionary"])
    )
    board = cv2.aruco.CharucoBoard(
        (int(board_cfg["squares_x"]), int(board_cfg["squares_y"])),
        float(board_cfg["square_length"]),
        float(board_cfg["marker_length"]),
        dictionary,
    )
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector_params.cornerRefinementWinSize = 5
    detector_params.adaptiveThreshWinSizeMin = 3
    detector_params.adaptiveThreshWinSizeMax = 53
    detector_params.adaptiveThreshWinSizeStep = 4
    detector_params.minMarkerPerimeterRate = 0.015
    charuco_params = cv2.aruco.CharucoParameters()
    charuco_params.tryRefineMarkers = True
    return board, cv2.aruco.CharucoDetector(board, charuco_params, detector_params)


def _intrinsics(scene, cfg, samples, board, detector):
    detections, size = [], None
    for sample in samples:
        image = cv2.imread(str(scene / sample["rgb"]))
        if image is None:
            continue
        current = (image.shape[1], image.shape[0])
        if size is not None and current != size:
            raise ValueError("数据集中存在不同图像尺寸")
        size = current
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is not None and len(ids) >= int(cfg["charuco"]["min_corners"]):
            detections.append((corners, ids))
    if not detections:
        raise RuntimeError("没有可用于内参计算的 ChArUco 图像")
    rms, matrix, distortion, _, _ = cv2.aruco.calibrateCameraCharuco(
        [x[0] for x in detections], [x[1] for x in detections], board,
        size, None, None,
    )
    return matrix, distortion, float(rms), size


def _node_residual(base_to_gripper, camera_to_board, base_to_camera):
    constants = [
        np.linalg.inv(bg) @ base_to_camera @ cb
        for bg, cb in zip(base_to_gripper, camera_to_board)
    ]
    translations = np.asarray([x[:3, 3] for x in constants])
    rotations = Rotation.from_matrix([x[:3, :3] for x in constants])
    mean_t = translations.mean(axis=0)
    mean_r = rotations.mean()
    t_mm = np.linalg.norm(translations - mean_t, axis=1) * 1000.0
    r_deg = np.degrees((mean_r.inv() * rotations).magnitude())
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(t_mm ** 2))),
        "translation_max_mm": float(t_mm.max()),
        "rotation_rms_deg": float(np.sqrt(np.mean(r_deg ** 2))),
        "rotation_max_deg": float(r_deg.max()),
    }


def calibrate_session(scene: Path, cfg: dict) -> dict:
    scene = Path(scene)
    samples = json.loads((scene / "samples.json").read_text(encoding="utf-8"))
    accepted = [x for x in samples if x.get("status", "accepted") == "accepted"]
    board, detector = make_board(cfg)
    camera_matrix, distortion, reprojection, image_size = _intrinsics(
        scene, cfg, accepted, board, detector
    )
    minimum = int(cfg["dynamic_eye_to_hand"]["min_samples_per_node"])
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method_name = str(cfg["calibration"]["hand_eye_method"]).upper()
    grouped = {}
    for item in accepted:
        grouped.setdefault(item["node_id"], []).append(item)
    nodes, skipped = [], []
    for node_id, items in sorted(grouped.items()):
        robot, camera, used = [], [], []
        for item in items:
            image = cv2.imread(str(scene / item["rgb"]))
            record = json.loads((scene / item["pose"]).read_text(encoding="utf-8"))
            corners, ids, _, _ = detector.detectBoard(image)
            if ids is None or len(ids) < int(cfg["charuco"]["min_corners"]):
                continue
            object_points, image_points = board.matchImagePoints(corners, ids)
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, camera_matrix, distortion
            )
            if not ok:
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            camera.append(transform(rotation, tvec))
            robot.append(pose_to_transform(record["pose_base_to_gripper_m_rad"]))
            used.append(item["index"])
        if len(used) < minimum:
            skipped.append({"node_id": node_id, "valid_samples": len(used), "required": minimum})
            continue
        # Eye-to-hand OpenCV convention: use gripper_T_base as robot input.
        gripper_to_base = [np.linalg.inv(value) for value in robot]
        r_out, t_out = cv2.calibrateHandEye(
            [x[:3, :3] for x in gripper_to_base],
            [x[:3, 3] for x in gripper_to_base],
            [x[:3, :3] for x in camera],
            [x[:3, 3] for x in camera],
            method=methods[method_name],
        )
        output = transform(r_out, t_out)
        head = items[0]["head_target"]
        actual = np.asarray([[x["head_actual"]["yaw"], x["head_actual"]["pitch"]] for x in items])
        nodes.append({
            "node_id": node_id,
            "target_yaw": int(head["yaw"]), "target_pitch": int(head["pitch"]),
            "mean_actual_yaw": float(actual[:, 0].mean()),
            "mean_actual_pitch": float(actual[:, 1].mean()),
            "sample_indices": used,
            "base_to_camera": output.tolist(),
            "consistency": _node_residual(robot, camera, output),
        })
    if not nodes:
        raise RuntimeError(f"没有节点达到最低有效样本数 {minimum}")
    result = {
        "model": "moving_head_eye_to_hand_node_map",
        "transform_name": "arm_base_to_head_camera",
        "query_coordinates": "raw_servo_yaw_pitch",
        "interpolation": "inverse_distance_translation_and_weighted_quaternion",
        "extrapolation_allowed": False,
        "hand_eye_method": method_name,
        "image_size": list(image_size),
        "intrinsic_source": "charuco_full_dataset",
        "reprojection_rms_px": reprojection,
        "camera_matrix": camera_matrix.tolist(),
        "distortion": distortion.reshape(-1).tolist(),
        "nodes": nodes, "skipped_nodes": skipped,
    }
    output = scene / "results" / "dynamic_eye_to_hand_result.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def query_transform(result: dict, yaw: float, pitch: float, neighbors: int = 4):
    nodes = result["nodes"]
    points = np.asarray([[x["mean_actual_yaw"], x["mean_actual_pitch"]] for x in nodes])
    query = np.asarray([yaw, pitch], dtype=float)
    lower, upper = points.min(axis=0), points.max(axis=0)
    if np.any(query < lower) or np.any(query > upper):
        raise ValueError(f"查询点超出已标定范围 yaw={lower[0]:.1f}..{upper[0]:.1f}, pitch={lower[1]:.1f}..{upper[1]:.1f}")
    distance = np.linalg.norm(points - query, axis=1)
    if distance.min() < 1e-9:
        return np.asarray(nodes[int(distance.argmin())]["base_to_camera"]), [nodes[int(distance.argmin())]["node_id"]]
    selected = np.argsort(distance)[:min(neighbors, len(nodes))]
    weights = 1.0 / np.maximum(distance[selected], 1e-9) ** 2
    weights /= weights.sum()
    matrices = [np.asarray(nodes[i]["base_to_camera"]) for i in selected]
    output = np.eye(4)
    output[:3, 3] = np.sum([w * m[:3, 3] for w, m in zip(weights, matrices)], axis=0)
    output[:3, :3] = Rotation.from_matrix([m[:3, :3] for m in matrices]).mean(weights=weights).as_matrix()
    return output, [nodes[i]["node_id"] for i in selected]
