#!/usr/bin/env python3
"""LAN collector for eye-in-hand calibration with a fixed ChArUco board."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parent
BASE_REPO = ROOT.parent


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


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


def pose_rotation_deg(pose_a, pose_b):
    """Return relative magnitude for RealMan XYZ Euler poses (radians)."""
    r_a = Rotation.from_euler("xyz", pose_a[3:6], degrees=False)
    r_b = Rotation.from_euler("xyz", pose_b[3:6], degrees=False)
    return float(np.degrees((r_a.inv() * r_b).magnitude()))


class Runtime:
    def __init__(self, cfg, profile_name):
        self.cfg = cfg
        self.profile_name = profile_name
        self.profile = cfg["profiles"][profile_name]
        if self.profile.get("mode") != "eye_in_hand":
            raise ValueError(f"{profile_name} 不是眼在手上配置")
        data_root = Path(cfg["paths"]["data_root"])
        if not data_root.is_absolute():
            data_root = BASE_REPO / data_root
        self.scene = data_root / profile_name
        for name in ("rgb", "poses", "detections", "rejected", "logs", "results"):
            (self.scene / name).mkdir(parents=True, exist_ok=True)
        self.samples_path = self.scene / "samples.json"
        self.samples = json.loads(self.samples_path.read_text(encoding="utf-8")) if self.samples_path.exists() else []
        self.camera = self.arm = None
        self.connected = self.connecting = self.running = False
        self.connect_lock = threading.Lock()
        self.lock = threading.RLock()
        self.detection_lock = threading.Lock()
        self.calibration_lock = threading.Lock()
        self.latest_image = self.latest_annotated = self.latest_jpeg = None
        self.frame_timestamp_ms = None
        self.frame_host_timestamp_ns = None
        self.quality = {"ok": False, "reason": "尚未连接硬件", "corners": 0}
        self.arm_state = None
        self.work_frame = None
        self.intrinsics = None
        self.camera_matrix = None
        self.distortion = None
        self.last_error = ""
        self.board, self.detector = make_board(cfg)
        self.target_samples = 25

    def accepted_samples(self):
        return [x for x in self.samples if x.get("status", "accepted") == "accepted"]

    def _load_pose_records(self):
        records = []
        for item in self.accepted_samples():
            path = self.scene / item.get("pose", "")
            if not path.exists():
                continue
            try:
                records.append(json.loads(path.read_text(encoding="utf-8")))
            except Exception:
                pass
        return records

    def diversity(self):
        records = self._load_pose_records()
        if len(records) < 2:
            return {"samples": len(records), "translation_span_mm": 0.0, "rotation_span_deg": 0.0,
                    "hint": "至少采集2组后显示姿态变化"}
        poses = [r["pose_base_to_gripper_m_rad"] for r in records]
        xyz = np.asarray([p[:3] for p in poses], dtype=float)
        translation_span = float(np.linalg.norm(xyz.max(axis=0) - xyz.min(axis=0)) * 1000.0)
        rotation_span = max(pose_rotation_deg(poses[i], poses[j]) for i in range(len(poses)) for j in range(i))
        hints = []
        if translation_span < 80.0:
            hints.append("增加距离和图像位置变化")
        if rotation_span < 25.0:
            hints.append("增加绕X/Y/Z方向旋转")
        return {"samples": len(records), "translation_span_mm": round(translation_span, 1),
                "rotation_span_deg": round(rotation_span, 1),
                "hint": "；".join(hints) if hints else "姿态跨度已有明显变化，请继续覆盖多个旋转轴"}

    def status(self):
        arm = None
        if self.arm_state is not None:
            arm = {"joints_deg": self.arm_state.joints, "pose": self.arm_state.pose.as_list(),
                   "errors": self.arm_state.err}
        return {
            "connected": self.connected, "connecting": self.connecting, "profile": self.profile_name,
            "arm_side": self.profile["arm_side"], "arm_ip": str(self.profile["arm_ip"]),
            "camera_serial_expected": str(self.profile["camera_serial"]),
            "intrinsics": self.intrinsics, "arm": arm, "quality": self.quality,
            "sample_count": len(self.accepted_samples()), "sample_target": self.target_samples,
            "diversity": self.diversity(), "last_error": self.last_error,
        }

    def connect(self):
        if not self.connect_lock.acquire(blocking=False):
            raise RuntimeError("硬件正在连接，请勿重复点击")
        if self.connected:
            self.connect_lock.release()
            return self.status()
        self.connecting = True
        paths = self.cfg["paths"]
        for path in (paths["camera_api"], paths["arm_api"]):
            if path not in sys.path:
                sys.path.insert(0, path)
        from D435_rgb_depth import D435Camera
        from realman_arm_api_api2 import RealmanArmClient
        cap = self.cfg["capture"]
        self.camera = D435Camera(
            serial=str(self.profile["camera_serial"]), width=int(cap["width"]), height=int(cap["height"]),
            fps=int(cap["fps"]), enable_color=True, enable_depth=False, align_depth_to_color=False,
        )
        self.arm = RealmanArmClient(ip=str(self.profile["arm_ip"]), model=self.profile["arm_side"])
        try:
            self.arm.connect()
            self.work_frame = self.arm.raw_call("rm_get_current_work_frame")
            work_pose = self.work_frame.get("pose", []) if isinstance(self.work_frame, dict) else []
            if work_pose and any(abs(float(v)) > 1e-9 for v in work_pose):
                raise RuntimeError(f"当前工作坐标系不与机械臂基座重合: {self.work_frame}")
            self.camera.start()
            for _ in range(int(cap["warmup_frames"])):
                self.camera.get_frames(timeout_ms=5000)
            intrinsics = self.camera.get_intrinsics("color")
            self.intrinsics = intrinsics
            self.camera_matrix = np.array([
                [intrinsics["fx"], 0.0, intrinsics["ppx"]],
                [0.0, intrinsics["fy"], intrinsics["ppy"]],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            self.distortion = np.asarray(intrinsics["coeffs"], dtype=np.float64)
            atomic_json(self.scene / f"camera_intrinsics_factory_{intrinsics['width']}x{intrinsics['height']}.json", intrinsics)
            atomic_json(self.scene / "session.json", {
                "profile_name": self.profile_name, "profile": self.profile,
                "charuco": self.cfg["charuco"], "capture": cap,
                "camera_intrinsics_factory": intrinsics,
                "fixed_target_instruction": "ChArUco板固定在机器人外部，采集过程中不得移动",
            })
            self.connected = self.running = True
            threading.Thread(target=self._camera_loop, daemon=True).start()
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
            try:
                self.camera.stop()
            except Exception:
                pass
        if self.arm:
            try:
                self.arm.disconnect()
            except Exception:
                pass
        self.camera = self.arm = None

    @staticmethod
    def _gamma_image(gray):
        mean = float(gray.mean()) / 255.0
        gamma = 1.0 if mean <= 0.01 or mean >= 0.99 else float(np.clip(np.log(0.5) / np.log(mean), 0.55, 1.60))
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
        cv2.cornerSubPix(original_gray, corners, (5, 5), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        hull = cv2.convexHull(corners.reshape(-1, 2))
        coverage = float(cv2.contourArea(hull) / (original_gray.shape[0] * original_gray.shape[1]))
        reprojection = None
        if self.camera_matrix is not None:
            object_points, image_points = self.board.matchImagePoints(corners, ids)
            ok, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.distortion,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.distortion)
            delta = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
            reprojection = float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))
        score = len(ids) + 120.0 * coverage - 8.0 * (reprojection if reprojection is not None else 0.0)
        return {"corners": corners, "ids": ids, "method": method, "coverage": coverage,
                "reprojection_rms_px": reprojection, "score": score}

    def _best_detection(self, gray):
        with self.detection_lock:
            original = self._candidate(gray, gray, "gray")
            if original is not None and len(original["ids"]) >= 102 and (
                    original["reprojection_rms_px"] is None or original["reprojection_rms_px"] <= 1.0):
                return original
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
            gamma_image, gamma = self._gamma_image(gray)
            candidates = [original, self._candidate(clahe, gray, "clahe"),
                          self._candidate(gamma_image, gray, f"gamma_{gamma:.2f}")]
        candidates = [candidate for candidate in candidates if candidate is not None]
        return max(candidates, key=lambda candidate: candidate["score"]) if candidates else None

    def _evaluate(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = float(gray.mean())
        blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        selected = self._best_detection(gray)
        count = 0 if selected is None else len(selected["ids"])
        annotated = image.copy()
        if selected is not None:
            cv2.aruco.drawDetectedCornersCharuco(annotated, selected["corners"], selected["ids"])
        limits = self.cfg.get("eye_in_hand_web", {}).get("quality", self.cfg["dynamic_eye_to_hand"]["quality"])
        reasons = []
        if brightness < float(limits["min_brightness"]):
            reasons.append("图像过暗")
        if blur < float(limits["min_laplacian_variance"]):
            reasons.append("图像模糊")
        if count < int(self.cfg["charuco"]["min_corners"]):
            reasons.append("ChArUco角点不足")
        return annotated, {
            "ok": not reasons, "reason": "正常" if not reasons else "；".join(reasons),
            "corners": count, "brightness": round(brightness, 1), "blur_score": round(blur, 1),
            "detection_method": None if selected is None else selected["method"],
            "corner_coverage": None if selected is None else round(selected["coverage"], 4),
            "reprojection_rms_px": None if selected is None or selected["reprojection_rms_px"] is None
            else round(selected["reprojection_rms_px"], 4),
        }

    def _camera_loop(self):
        settings = self.cfg.get("eye_in_hand_web", {})
        preview_width = int(settings.get("preview_width", 960))
        jpeg_quality = int(settings.get("preview_jpeg_quality", 75))
        while self.running:
            try:
                frame = self.camera.get_frames(timeout_ms=3000)
                annotated, quality = self._evaluate(frame["color"])
                if annotated.shape[1] > preview_width:
                    height = round(annotated.shape[0] * preview_width / annotated.shape[1])
                    preview = cv2.resize(annotated, (preview_width, height), interpolation=cv2.INTER_AREA)
                else:
                    preview = annotated
                ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
                with self.lock:
                    self.latest_image = frame["color"].copy()
                    self.latest_annotated = annotated
                    self.latest_jpeg = encoded.tobytes() if ok else None
                    self.frame_timestamp_ms = frame["timestamp_ms"]
                    self.frame_host_timestamp_ns = time.time_ns()
                    self.quality = quality
            except Exception as exc:
                self.last_error = f"相机采集失败: {exc}"
                time.sleep(0.05)

    def _telemetry_loop(self):
        while self.running:
            try:
                self.arm_state = self.arm.get_state()
                self.last_error = ""
            except Exception as exc:
                self.last_error = f"机械臂读取失败: {exc}"
            time.sleep(0.1)

    def _next_index(self):
        values = [int(x.get("index", 0)) for x in self.samples]
        for directory in (self.scene / "rgb", self.scene / "poses", self.scene / "rejected"):
            for path in directory.iterdir():
                match = re.search(r"(\d{6})", path.name)
                if match:
                    values.append(int(match.group(1)))
        return max(values, default=0) + 1

    def capture(self):
        if not self.connected:
            raise RuntimeError("尚未连接硬件")
        with self.lock:
            if self.latest_image is None:
                raise RuntimeError("尚无相机图像")
            image = self.latest_image.copy()
            frame_timestamp_ms = self.frame_timestamp_ms
            frame_host_timestamp_ns = self.frame_host_timestamp_ns
        annotated, final_quality = self._evaluate(image)
        if not final_quality["ok"]:
            raise ValueError(f"锁定帧质量检查未通过: {final_quality['reason']}")
        host_before_ns = time.time_ns()
        state = self.arm.get_state()
        host_after_ns = time.time_ns()
        errors = [str(value) for value in state.err.get("err", [])]
        if any(value != "0" for value in errors):
            raise ValueError(f"机械臂错误: {state.err}")
        index = self._next_index()
        stem = f"{index:06d}"
        record = {
            "index": index, "profile": self.profile_name, "mode": "eye_in_hand",
            "arm_side": self.profile["arm_side"], "arm_ip": str(self.profile["arm_ip"]),
            "camera_serial": str(self.profile["camera_serial"]),
            "camera_timestamp_ms": frame_timestamp_ms,
            "frame_host_timestamp_ns": frame_host_timestamp_ns,
            "host_before_ns": host_before_ns, "host_after_ns": host_after_ns,
            "work_frame": self.work_frame, "joints_deg": state.joints,
            "pose_base_to_gripper_m_rad": state.pose.as_list(), "arm_errors": state.err,
            "quality": final_quality, "charuco_corner_count": final_quality["corners"],
        }
        image_tmp = self.scene / "rgb" / f".{stem}.png"
        detection_tmp = self.scene / "detections" / f".{stem}.png"
        pose_tmp = self.scene / "poses" / f".{stem}.json"
        if not cv2.imwrite(str(image_tmp), image) or not cv2.imwrite(str(detection_tmp), annotated):
            raise RuntimeError("图像写入失败")
        atomic_json(pose_tmp, record)
        image_path = self.scene / "rgb" / f"{stem}.png"
        detection_path = self.scene / "detections" / f"{stem}.png"
        pose_path = self.scene / "poses" / f"{stem}.json"
        image_tmp.replace(image_path)
        detection_tmp.replace(detection_path)
        pose_tmp.replace(pose_path)
        item = {"index": index, "rgb": f"rgb/{stem}.png", "pose": f"poses/{stem}.json",
                "detection": f"detections/{stem}.png", "status": "accepted"}
        self.samples.append(item)
        atomic_json(self.samples_path, self.samples)
        return {**item, "quality": final_quality, "diversity": self.diversity()}

    def undo(self):
        accepted = self.accepted_samples()
        if not accepted:
            raise ValueError("没有可撤回样本")
        item = accepted[-1]
        item["status"] = "rejected"
        item["rejected_reason"] = "undo"
        item["rejected_at_ns"] = time.time_ns()
        for key in ("rgb", "pose", "detection"):
            source = self.scene / item[key]
            if source.exists():
                target = self.scene / "rejected" / f"{item['index']:06d}_{source.name}"
                if target.exists():
                    target = self.scene / "rejected" / f"{time.time_ns()}_{source.name}"
                shutil.move(str(source), str(target))
        atomic_json(self.samples_path, self.samples)
        return item

    def reject_current(self):
        with self.lock:
            if self.latest_image is None:
                raise RuntimeError("尚无相机图像")
            image, quality = self.latest_image.copy(), dict(self.quality)
        stamp = time.time_ns()
        image_path = self.scene / "rejected" / f"preview_{stamp}.png"
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError("拒绝图像保存失败")
        record = {"type": "preview_rejection", "profile": self.profile_name,
                  "host_timestamp_ns": stamp, "quality": quality, "image": image_path.name}
        atomic_json(self.scene / "rejected" / f"preview_{stamp}.json", record)
        return record

    def calibrate(self):
        if not self.calibration_lock.acquire(blocking=False):
            raise RuntimeError("标定正在计算中")
        try:
            completed = len(self.accepted_samples())
            minimum = int(self.cfg["calibration"]["min_samples"])
            if completed < minimum:
                raise ValueError(f"有效样本只有{completed}组，至少需要{minimum}组")
            command = [sys.executable, str(BASE_REPO / "compute_calibration.py"), self.profile_name]
            result = subprocess.run(command, cwd=str(BASE_REPO), capture_output=True, text=True, timeout=600)
            log = {"command": command, "returncode": result.returncode, "stdout": result.stdout,
                   "stderr": result.stderr, "finished_at_ns": time.time_ns()}
            atomic_json(self.scene / "logs" / f"calibration_{log['finished_at_ns']}.json", log)
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "标定计算失败")
            source = self.scene / "calibration_result.json"
            if not source.exists():
                raise RuntimeError("计算结束但未找到calibration_result.json")
            value = json.loads(source.read_text(encoding="utf-8"))
            result_path = self.scene / "results" / "calibration_result.json"
            atomic_json(result_path, value)
            return value
        finally:
            self.calibration_lock.release()


def build_app(config_path, profile_name):
    cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    runtime = Runtime(cfg, profile_name)
    side_name = "左臂" if runtime.profile["arm_side"] == "left" else "右臂"
    app = FastAPI(title=f"{side_name}眼在手上标定")

    @app.get("/")
    def index():
        return FileResponse(ROOT / "static" / "index.html")

    @app.get("/api/status")
    def status():
        return runtime.status()

    @app.post("/api/connect")
    def connect():
        try:
            return runtime.connect()
        except Exception as exc:
            raise HTTPException(500, str(exc))

    @app.post("/api/disconnect")
    def disconnect():
        runtime.disconnect()
        return {"ok": True}

    @app.post("/api/capture")
    def capture():
        try:
            return runtime.capture()
        except Exception as exc:
            raise HTTPException(400, str(exc))

    @app.post("/api/undo")
    def undo():
        try:
            return runtime.undo()
        except Exception as exc:
            raise HTTPException(400, str(exc))

    @app.post("/api/reject")
    def reject():
        try:
            return runtime.reject_current()
        except Exception as exc:
            raise HTTPException(400, str(exc))

    @app.post("/api/calibrate")
    def calibrate():
        try:
            return runtime.calibrate()
        except Exception as exc:
            raise HTTPException(400, str(exc))

    @app.get("/api/video")
    def video():
        preview_fps = int(cfg.get("eye_in_hand_web", {}).get("preview_fps", 15))
        preview_period = 1.0 / max(1, preview_fps)

        def stream():
            last_frame = None
            while True:
                with runtime.lock:
                    frame = runtime.latest_jpeg
                if frame is not None and frame is not last_frame:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                           str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                    last_frame = frame
                time.sleep(preview_period)

        return StreamingResponse(stream(), media_type="multipart/x-mixed-replace; boundary=frame",
                                 headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                                          "Pragma": "no-cache", "X-Accel-Buffering": "no"})

    @app.on_event("shutdown")
    def shutdown():
        runtime.disconnect()

    return app


if __name__ == "__main__":
    raise SystemExit("请使用 start_left_eye_in_hand_web.py 或 start_right_eye_in_hand_web.py")
