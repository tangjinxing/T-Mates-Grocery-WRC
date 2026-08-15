"""Capture one aligned RGB-D frame from a D435 (head / left / right)."""

from __future__ import annotations

import os
import sys
import time
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

CAMERA_API = os.getenv("CAMERA_API_PATH", "/home/lh/robot_api/camera_api")
CAMERA_SERIALS = {
    "head": os.getenv("CAMERA_SERIAL_HEAD", "344422070170"),
    "left": os.getenv("CAMERA_SERIAL_LEFT", "215322079194"),
    "right": os.getenv("CAMERA_SERIAL_RIGHT", "335522072306"),
}


@dataclass
class RgbdFrame:
    camera: str
    serial: str
    color: np.ndarray
    depth: np.ndarray
    depth_scale_m: float
    intrinsics: dict[str, Any]
    captured_at: str

    @property
    def width(self) -> int:
        return int(self.color.shape[1])

    @property
    def height(self) -> int:
        return int(self.color.shape[0])

    def camera_json(self) -> dict[str, Any]:
        """BOP-style meta: depth_mm = depth_raw * depth_scale."""

        k = self.intrinsics
        return {
            "camera": self.camera,
            "camera_serial": self.serial,
            "captured_at": self.captured_at,
            "image_width": self.width,
            "image_height": self.height,
            "cam_K": [
                float(k["fx"]),
                0.0,
                float(k["ppx"]),
                0.0,
                float(k["fy"]),
                float(k["ppy"]),
                0.0,
                0.0,
                1.0,
            ],
            "depth_scale": round(float(self.depth_scale_m) * 1000.0, 6),
            "depth_scale_m": float(self.depth_scale_m),
            "aligned_to_color": True,
        }

    def rgb_png(self) -> bytes:
        ok, encoded = cv2.imencode(".png", self.color)
        if not ok:
            raise RuntimeError(f"{self.camera} RGB PNG 编码失败")
        return encoded.tobytes()

    def rgb_jpeg(self, quality: int = 90) -> bytes:
        ok, encoded = cv2.imencode(
            ".jpg", self.color, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not ok:
            raise RuntimeError(f"{self.camera} JPEG 编码失败")
        return encoded.tobytes()

    def depth_png(self) -> bytes:
        if self.depth.dtype != np.uint16:
            raise RuntimeError(f"{self.camera} 深度类型不是 uint16: {self.depth.dtype}")
        ok, encoded = cv2.imencode(".png", self.depth)
        if not ok:
            raise RuntimeError(f"{self.camera} 深度 PNG 编码失败")
        return encoded.tobytes()

    def save(self, out_dir: Path) -> dict[str, Path]:
        out_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = out_dir / "rgb.png"
        depth_path = out_dir / "depth.png"
        meta_path = out_dir / "camera.json"
        if not cv2.imwrite(str(rgb_path), self.color):
            raise RuntimeError(f"RGB 保存失败: {rgb_path}")
        if not cv2.imwrite(str(depth_path), self.depth):
            raise RuntimeError(f"深度保存失败: {depth_path}")
        meta_path.write_text(
            json.dumps(self.camera_json(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return {"rgb": rgb_path, "depth": depth_path, "camera": meta_path}


def _import_d435():
    if CAMERA_API not in sys.path:
        sys.path.insert(0, CAMERA_API)
    from D435_rgb_depth import D435Camera, list_realsense_devices

    return D435Camera, list_realsense_devices


def capture_rgbd(
    camera: str,
    *,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    warmup: int = 20,
) -> RgbdFrame:
    if camera not in CAMERA_SERIALS:
        raise ValueError(f"unknown camera={camera!r}; expected {sorted(CAMERA_SERIALS)}")

    D435Camera, list_realsense_devices = _import_d435()
    serial = CAMERA_SERIALS[camera]
    present = {str(item.get("serial", "")) for item in list_realsense_devices()}
    if serial not in present:
        raise RuntimeError(
            f"{camera} 相机 {serial} 未找到，当前设备: {sorted(s for s in present if s)}"
        )

    cam = D435Camera(
        serial=serial,
        width=width,
        height=height,
        fps=fps,
        enable_color=True,
        enable_depth=True,
        align_depth_to_color=True,
    )
    cam.start()
    try:
        packet = None
        for _ in range(max(warmup, 1) + 1):
            packet = cam.get_frames(timeout_ms=5000)
        if packet is None or packet.get("color") is None or packet.get("depth") is None:
            raise RuntimeError(f"{camera} 未能同时取得 RGB 和深度")
        color = packet["color"]
        depth = packet["depth"]
        if color.shape[:2] != depth.shape[:2]:
            raise RuntimeError(
                f"{camera} RGB/深度尺寸不一致: {color.shape} / {depth.shape}"
            )
        if depth.dtype != np.uint16:
            raise RuntimeError(f"{camera} 深度类型不是 uint16: {depth.dtype}")
        scale = packet.get("depth_scale")
        if scale is None or float(scale) <= 0:
            raise RuntimeError(f"{camera} depth_scale 无效: {scale}")
        return RgbdFrame(
            camera=camera,
            serial=serial,
            color=color,
            depth=depth,
            depth_scale_m=float(scale),
            intrinsics=cam.get_intrinsics("color"),
            captured_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
        )
    finally:
        cam.stop()
        time.sleep(0.05)
