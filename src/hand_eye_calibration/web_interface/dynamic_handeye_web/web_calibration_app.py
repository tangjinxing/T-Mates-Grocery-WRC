#!/usr/bin/env python3
"""LAN web collector for moving-head eye-to-hand calibration."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from dynamic_calibration import calibrate_session, make_board, query_transform

ROOT = Path(__file__).resolve().parent
BASE_REPO = ROOT.parent


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


class HeadTarget(BaseModel):
    yaw: int
    pitch: int


class QueryTarget(BaseModel):
    yaw: float
    pitch: float


class Runtime:
    def __init__(self, cfg, profile_name):
        self.cfg, self.profile_name = cfg, profile_name
        self.profile = cfg["profiles"][profile_name]
        data_root = Path(cfg["paths"]["data_root"])
        if not data_root.is_absolute():
            data_root = BASE_REPO / data_root
        self.scene = data_root / f"dynamic_{profile_name}"
        for name in ("rgb", "poses", "detections", "rejected", "logs", "results"):
            (self.scene / name).mkdir(parents=True, exist_ok=True)
        self.samples_path = self.scene / "samples.json"
        self.samples = json.loads(self.samples_path.read_text(encoding="utf-8")) if self.samples_path.exists() else []
        self.nodes = cfg["dynamic_eye_to_hand"]["nodes"]
        self.current_node = 0
        self.camera = self.arm = self.head = None
        self.connected = False
        self.connecting = False
        self.connect_lock = threading.Lock()
        self.running = False
        self.lock = threading.RLock()
        self.latest_image = self.latest_annotated = self.latest_jpeg = None
        self.frame_timestamp_ms = None
        self.quality = {"ok": False, "reason": "尚未连接硬件", "corners": 0}
        self.head_actual = {"yaw": None, "pitch": None}
        self.arm_state = None
        self.last_error = ""
        self.board, self.detector = make_board(cfg)
        self.detection_lock = threading.Lock()
        self.camera_matrix = None
        self.distortion = None

    def counts(self):
        accepted = [x for x in self.samples if x.get("status", "accepted") == "accepted"]
        node_id = self.nodes[self.current_node]["id"]
        return len([x for x in accepted if x["node_id"] == node_id]), len(accepted)

    def status(self):
        node_count, total = self.counts()
        arm = None
        if self.arm_state is not None:
            arm = {"joints_deg": self.arm_state.joints, "pose": self.arm_state.pose.as_list(), "errors": self.arm_state.err}
        return {
            "connected": self.connected, "connecting": self.connecting, "profile": self.profile_name,
            "current_node_index": self.current_node, "current_node": self.nodes[self.current_node],
            "node_count": node_count, "node_target": int(self.cfg["dynamic_eye_to_hand"]["samples_per_node"]),
            "total_count": total, "total_target": len(self.nodes) * int(self.cfg["dynamic_eye_to_hand"]["samples_per_node"]),
            "head_actual": self.head_actual, "arm": arm, "quality": self.quality,
            "last_error": self.last_error,
        }

    def connect(self):
        if not self.connect_lock.acquire(blocking=False):
            raise RuntimeError("硬件正在连接，请勿重复点击")
        if self.connected:
            self.connect_lock.release()
            return self.status()
        self.connecting = True
        paths = self.cfg["paths"]
        for path in (paths["camera_api"], paths["arm_api"], paths["head_api"]):
            if path not in sys.path:
                sys.path.insert(0, path)
        from D435_rgb_depth import D435Camera
        from realman_arm_api_api2 import RealmanArmClient
        from servo_api import HeadControlSDK
        cap = self.cfg["capture"]
        self.camera = D435Camera(serial=str(self.profile["camera_serial"]), width=int(cap["width"]), height=int(cap["height"]), fps=int(cap["fps"]), enable_color=True, enable_depth=False, align_depth_to_color=False)
        self.arm = RealmanArmClient(ip=str(self.profile["arm_ip"]), model=self.profile["arm_side"])
        self.head = HeadControlSDK(str(self.cfg["dynamic_eye_to_hand"]["head_port"]), int(self.cfg["dynamic_eye_to_hand"]["head_baudrate"]))
        try:
            self.arm.connect()
            if not self.head.connect():
                raise RuntimeError("头部串口连接失败")
            self.camera.start()
            for _ in range(int(cap["warmup_frames"])):
                self.camera.get_frames(timeout_ms=5000)
            intrinsics = self.camera.get_intrinsics("color")
            self.camera_matrix = np.array([
                [intrinsics["fx"], 0.0, intrinsics["ppx"]],
                [0.0, intrinsics["fy"], intrinsics["ppy"]],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            self.distortion = np.asarray(intrinsics["coeffs"], dtype=np.float64)
            atomic_json(self.scene / f"camera_intrinsics_factory_{intrinsics['width']}x{intrinsics['height']}.json", intrinsics)
            atomic_json(self.scene / "session.json", {"profile": self.profile, "charuco": self.cfg["charuco"], "capture": cap, "dynamic_eye_to_hand": self.cfg["dynamic_eye_to_hand"]})
            self.connected = self.running = True
            threading.Thread(target=self._loop, daemon=True).start()
            threading.Thread(target=self._telemetry_loop, daemon=True).start()
            return self.status()
        except Exception:
            self.disconnect()
            raise
        finally:
            self.connecting = False
            self.connect_lock.release()

    def disconnect(self):
        self.running = self.connected = False
        if self.camera:
            try: self.camera.stop()
            except Exception: pass
        if self.arm:
            try: self.arm.disconnect()
            except Exception: pass
        if self.head:
            try: self.head.disconnect()
            except Exception: pass

    @staticmethod
    def _gamma_image(gray):
        mean = float(gray.mean()) / 255.0
        if mean <= 0.01 or mean >= 0.99:
            gamma = 1.0
        else:
            gamma = float(np.clip(np.log(0.5) / np.log(mean), 0.55, 1.60))
        table = np.clip((np.arange(256, dtype=np.float32) / 255.0) ** gamma * 255.0, 0, 255).astype(np.uint8)
        return cv2.LUT(gray, table), gamma

    def _candidate(self, detection_image, original_gray, method):
        corners, ids, _, _ = self.detector.detectBoard(detection_image)
        if ids is None or corners is None or len(ids) < 4:
            return None
        ids = np.asarray(ids, dtype=np.int32).reshape(-1, 1)
        corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        flat_ids = ids.reshape(-1)
        maximum_id = (int(self.cfg["charuco"]["squares_x"]) - 1) * (int(self.cfg["charuco"]["squares_y"]) - 1)
        if len(np.unique(flat_ids)) != len(flat_ids) or np.any(flat_ids < 0) or np.any(flat_ids >= maximum_id):
            return None
        # Detection can use an enhanced image, but final localization is always on raw grayscale.
        cv2.cornerSubPix(
            original_gray, corners, (5, 5), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        hull = cv2.convexHull(corners.reshape(-1, 2))
        coverage = float(cv2.contourArea(hull) / (original_gray.shape[0] * original_gray.shape[1]))
        reprojection = None
        if self.camera_matrix is not None:
            object_points, image_points = self.board.matchImagePoints(corners, ids)
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.distortion,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                return None
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.distortion)
            delta = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
            reprojection = float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))
        # Count and spatial coverage reward robust poses; reprojection rejects inaccurate candidates.
        score = len(ids) + 120.0 * coverage - 8.0 * (reprojection if reprojection is not None else 0.0)
        return {"corners": corners, "ids": ids, "method": method, "coverage": coverage,
                "reprojection_rms_px": reprojection, "score": score}

    def _best_detection(self, image, gray):
        with self.detection_lock:
            original = self._candidate(gray, gray, "gray")
            # A nearly complete, geometrically accurate raw result needs no fallback.
            if original is not None and len(original["ids"]) >= 102 and (
                original["reprojection_rms_px"] is None or original["reprojection_rms_px"] <= 1.0
            ):
                return original
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
            gamma_image, gamma = self._gamma_image(gray)
            candidates = [
                original,
                self._candidate(clahe, gray, "clahe"),
                self._candidate(gamma_image, gray, f"gamma_{gamma:.2f}"),
            ]
        candidates = [value for value in candidates if value is not None]
        return max(candidates, key=lambda value: value["score"]) if candidates else None

    def _evaluate(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = float(gray.mean())
        blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        selected = self._best_detection(image, gray)
        count = 0 if selected is None else len(selected["ids"])
        annotated = image.copy()
        if selected is not None:
            cv2.aruco.drawDetectedCornersCharuco(annotated, selected["corners"], selected["ids"])
        limits = self.cfg["dynamic_eye_to_hand"]["quality"]
        reasons = []
        if brightness < float(limits["min_brightness"]): reasons.append("图像过暗")
        if blur < float(limits["min_laplacian_variance"]): reasons.append("图像模糊")
        if count < int(self.cfg["charuco"]["min_corners"]): reasons.append("ChArUco角点不足")
        return annotated, {
            "ok": not reasons, "reason": "正常" if not reasons else "；".join(reasons),
            "corners": count, "brightness": round(brightness, 1), "blur_score": round(blur, 1),
            "detection_method": None if selected is None else selected["method"],
            "corner_coverage": None if selected is None else round(selected["coverage"], 4),
            "reprojection_rms_px": None if selected is None or selected["reprojection_rms_px"] is None else round(selected["reprojection_rms_px"], 4),
        }

    def _loop(self):
        preview_width = int(self.cfg["dynamic_eye_to_hand"].get("preview_width", 960))
        while self.running:
            try:
                frame = self.camera.get_frames(timeout_ms=3000)
                annotated, quality = self._evaluate(frame["color"])
                if annotated.shape[1] > preview_width:
                    preview_height = round(annotated.shape[0] * preview_width / annotated.shape[1])
                    preview = cv2.resize(annotated, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
                else:
                    preview = annotated
                ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, int(self.cfg["dynamic_eye_to_hand"]["preview_jpeg_quality"])])
                with self.lock:
                    self.latest_image = frame["color"].copy()
                    self.latest_annotated = annotated
                    self.latest_jpeg = encoded.tobytes() if ok else None
                    self.frame_timestamp_ms = frame["timestamp_ms"]
                    self.quality = quality
            except Exception as exc:
                self.last_error = f"相机采集失败: {exc}"
                time.sleep(0.05)

    def _telemetry_loop(self):
        """Poll slow serial/network devices without ever delaying camera frames."""
        while self.running:
            errors = []
            try:
                positions = self.head.read_positions([1, 2], timeout=0.15)
                if 1 in positions and 2 in positions:
                    self.head_actual = {"pitch": positions[1], "yaw": positions[2]}
                else:
                    errors.append("头部位置读取无响应")
            except Exception as exc:
                errors.append(f"头部读取失败: {exc}")
            try:
                self.arm_state = self.arm.get_state()
            except Exception as exc:
                errors.append(f"机械臂读取失败: {exc}")
            self.last_error = "；".join(errors)
            time.sleep(0.1)

    def move_head(self, target):
        limits = self.cfg["dynamic_eye_to_hand"]["safe_limits"]
        if not (limits["yaw"][0] <= target.yaw <= limits["yaw"][1] and limits["pitch"][0] <= target.pitch <= limits["pitch"][1]):
            raise ValueError(f"目标超出安全范围: {limits}")
        # The controller can drop back-to-back packets; separate both axes.
        if not self.head.rotate(1, target.pitch):
            raise RuntimeError("头部移动指令发送失败")
        time.sleep(0.12)
        if not self.head.rotate(2, target.yaw):
            raise RuntimeError("头部移动指令发送失败")
        tolerance = int(self.cfg["dynamic_eye_to_hand"]["position_tolerance"])
        actual = {}
        for attempt in range(30):
            time.sleep(0.1)
            actual = self.head.read_positions([1, 2], timeout=0.25)
            pitch_ok = 1 in actual and abs(actual[1] - target.pitch) <= tolerance
            yaw_ok = 2 in actual and abs(actual[2] - target.yaw) <= tolerance
            if pitch_ok and yaw_ok:
                self.head_actual = {"pitch": actual[1], "yaw": actual[2]}
                return {"target": target.model_dump(), "actual": self.head_actual, "reached": True}
            # Retry only the axis which has not arrived, once after settling.
            if attempt == 10:
                if not pitch_ok:
                    self.head.rotate(1, target.pitch)
                    time.sleep(0.12)
                if not yaw_ok:
                    self.head.rotate(2, target.yaw)
        readable = {"pitch": actual.get(1), "yaw": actual.get(2)}
        raise RuntimeError(f"头部在3秒内未到位，目标 yaw={target.yaw}, pitch={target.pitch}，实际 {readable}")

    def capture(self):
        with self.lock:
            if self.latest_image is None:
                raise RuntimeError("尚无相机图像")
            image, quality = self.latest_image.copy(), dict(self.quality)
        if not quality["ok"]:
            raise ValueError(f"质量检查未通过: {quality['reason']}")
        node = self.nodes[self.current_node]
        actual = self.head.read_positions([1, 2], timeout=0.5)
        if 1 not in actual or 2 not in actual:
            raise RuntimeError("无法读取头部实际位置")
        tolerance = int(self.cfg["dynamic_eye_to_hand"]["position_tolerance"])
        if abs(actual[1] - int(node["pitch"])) > tolerance or abs(actual[2] - int(node["yaw"])) > tolerance:
            raise ValueError(f"头部未到当前节点，实际 yaw={actual[2]}, pitch={actual[1]}")
        state = self.arm.get_state()
        errors = [str(v) for v in state.err.get("err", [])]
        if any(x != "0" for x in errors):
            raise ValueError(f"机械臂错误: {state.err}")
        index = max([int(x["index"]) for x in self.samples], default=0) + 1
        stem = f"{index:06d}"
        annotated, final_quality = self._evaluate(image)
        if not final_quality["ok"]:
            raise ValueError(f"锁定帧质量检查未通过: {final_quality['reason']}")
        record = {
            "index": index, "node_id": node["id"], "profile": self.profile_name,
            "camera_timestamp_ms": self.frame_timestamp_ms, "host_timestamp_ns": time.time_ns(),
            "head_target": {"yaw": int(node["yaw"]), "pitch": int(node["pitch"])},
            "head_actual": {"yaw": int(actual[2]), "pitch": int(actual[1])},
            "pose_base_to_gripper_m_rad": state.pose.as_list(), "joints_deg": state.joints,
            "arm_errors": state.err, "quality": final_quality,
        }
        image_tmp = self.scene / "rgb" / f".{stem}.png"
        detect_tmp = self.scene / "detections" / f".{stem}.png"
        if not cv2.imwrite(str(image_tmp), image) or not cv2.imwrite(str(detect_tmp), annotated):
            raise RuntimeError("图像写入失败")
        pose_tmp = self.scene / "poses" / f".{stem}.json"
        atomic_json(pose_tmp, record)
        image_path, detect_path, pose_path = self.scene / "rgb" / f"{stem}.png", self.scene / "detections" / f"{stem}.png", self.scene / "poses" / f"{stem}.json"
        image_tmp.replace(image_path); detect_tmp.replace(detect_path); pose_tmp.replace(pose_path)
        item = {"index": index, "node_id": node["id"], "head_target": record["head_target"], "head_actual": record["head_actual"], "rgb": f"rgb/{stem}.png", "pose": f"poses/{stem}.json", "detection": f"detections/{stem}.png", "status": "accepted"}
        self.samples.append(item); atomic_json(self.samples_path, self.samples)
        return item

    def undo(self):
        accepted = [x for x in self.samples if x.get("status", "accepted") == "accepted"]
        if not accepted: raise ValueError("没有可撤回样本")
        item = accepted[-1]; item["status"] = "rejected"; item["rejected_reason"] = "undo"
        for key in ("rgb", "pose", "detection"):
            source = self.scene / item[key]
            if source.exists(): shutil.move(str(source), str(self.scene / "rejected" / f"{item['index']:06d}_{source.name}"))
        atomic_json(self.samples_path, self.samples)
        return item

    def reject_current(self):
        """Reject the current preview without creating a calibration sample."""
        with self.lock:
            if self.latest_image is None:
                raise RuntimeError("尚无相机图像")
            image, quality = self.latest_image.copy(), dict(self.quality)
        stamp = time.time_ns()
        image_path = self.scene / "rejected" / f"preview_{stamp}.png"
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError("拒绝图像保存失败")
        record = {
            "type": "preview_rejection", "host_timestamp_ns": stamp,
            "node_id": self.nodes[self.current_node]["id"], "quality": quality,
            "image": image_path.name,
        }
        atomic_json(self.scene / "rejected" / f"preview_{stamp}.json", record)
        return record


def build_app(config_path, profile_name):
    cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    runtime = Runtime(cfg, profile_name)
    app = FastAPI(title="动态头部眼在手外标定")

    @app.get("/")
    def index(): return FileResponse(ROOT / "static" / "index.html")
    @app.get("/api/status")
    def status(): return runtime.status()
    @app.post("/api/connect")
    def connect():
        try: return runtime.connect()
        except Exception as exc: raise HTTPException(500, str(exc))
    @app.post("/api/disconnect")
    def disconnect(): runtime.disconnect(); return {"ok": True}
    @app.post("/api/head/move")
    def move(target: HeadTarget):
        try: return runtime.move_head(target)
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.post("/api/head/current-node")
    def move_current():
        node = runtime.nodes[runtime.current_node]
        try: return runtime.move_head(HeadTarget(yaw=node["yaw"], pitch=node["pitch"]))
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.post("/api/capture")
    def capture():
        try: return runtime.capture()
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.post("/api/undo")
    def undo():
        try: return runtime.undo()
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.post("/api/reject")
    def reject():
        try: return runtime.reject_current()
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.post("/api/node/next")
    def next_node(): runtime.current_node = min(runtime.current_node + 1, len(runtime.nodes) - 1); return runtime.status()
    @app.post("/api/node/previous")
    def previous_node(): runtime.current_node = max(runtime.current_node - 1, 0); return runtime.status()
    @app.get("/api/video")
    def video():
        preview_period = 1.0 / max(1, int(cfg["dynamic_eye_to_hand"].get("preview_fps", 15)))
        def stream():
            last_frame = None
            while True:
                with runtime.lock: frame = runtime.latest_jpeg
                if frame is not None and frame is not last_frame:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                    last_frame = frame
                time.sleep(preview_period)
        return StreamingResponse(stream(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0", "Pragma": "no-cache", "X-Accel-Buffering": "no"})
    @app.post("/api/calibrate")
    def calibrate():
        try: return calibrate_session(runtime.scene, cfg)
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.post("/api/query-transform")
    def query(target: QueryTarget):
        path = runtime.scene / "results" / "dynamic_eye_to_hand_result.json"
        if not path.exists(): raise HTTPException(400, "请先完成标定计算")
        try:
            matrix, source_nodes = query_transform(json.loads(path.read_text(encoding="utf-8")), target.yaw, target.pitch)
            return {"yaw": target.yaw, "pitch": target.pitch, "base_to_camera": matrix.tolist(), "source_nodes": source_nodes}
        except Exception as exc: raise HTTPException(400, str(exc))
    @app.on_event("shutdown")
    def shutdown(): runtime.disconnect()
    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(BASE_REPO / "calibration_config.yaml"))
    parser.add_argument("--profile", choices=["eye_to_hand_left", "eye_to_hand_right"], default="eye_to_hand_right")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    uvicorn.run(build_app(args.config, args.profile), host=args.host, port=args.port)
