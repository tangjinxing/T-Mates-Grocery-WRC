from __future__ import annotations

import sys
import threading
import time
from typing import Any, Iterator

import cv2


class CameraManager:
    """Own exactly one active D435 and retain only its newest frame."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        sys.path.insert(0, config["paths"]["camera_api"])
        from D435_rgb_depth import D435Camera, list_realsense_devices

        self._camera_type = D435Camera
        self._list_devices = list_realsense_devices
        self._lock = threading.RLock()
        self._switch_lock = threading.Lock()
        self._camera = None
        self._active: str | None = None
        self._frame = None
        self._frame_id = 0
        self._timestamp_ms: float | None = None
        self._received_monotonic: float | None = None
        self._intrinsics: dict[str, Any] | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._error: str | None = None
        self._fps = 0.0
        self._available_cache: list[dict[str, Any]] = []
        self._available_checked = 0.0

    def _available_devices(self) -> list[dict[str, Any]]:
        now = time.monotonic()
        if now - self._available_checked < 2.0:
            return self._available_cache
        try:
            devices = self._list_devices()
            self._available_cache = devices
            self._available_checked = now
        except Exception as exc:
            self._error = f"相机枚举失败: {exc}"
        return self._available_cache

    def status(self) -> dict[str, Any]:
        devices = self._available_devices()
        serials = {str(item.get("serial", "")) for item in devices}
        configured = {
            name: {
                "label": item["label"],
                "serial": str(item["serial"]),
                "present": str(item["serial"]) in serials,
            }
            for name, item in self.config["cameras"].items()
        }
        with self._lock:
            latency = None
            if self._received_monotonic is not None:
                latency = round((time.monotonic() - self._received_monotonic) * 1000.0, 1)
            return {
                "configured": configured,
                "active": self._active,
                "streaming": self._camera is not None and self._thread is not None and self._thread.is_alive(),
                "frame_id": self._frame_id,
                "fps": round(self._fps, 1),
                "latest_frame_age_ms": latency,
                "timestamp_ms": self._timestamp_ms,
                "intrinsics": self._intrinsics,
                "error": self._error,
            }

    def switch(self, name: str) -> dict[str, Any]:
        if name not in self.config["cameras"]:
            raise ValueError(f"未知相机: {name}")
        with self._switch_lock:
            if self._active == name and self._camera is not None:
                return self.status()
            self._stop_locked()
            stream = self.config["camera_stream"]
            serial = str(self.config["cameras"][name]["serial"])
            camera = self._camera_type(
                serial=serial,
                width=int(stream["width"]),
                height=int(stream["height"]),
                fps=int(stream["fps"]),
                enable_color=True,
                enable_depth=False,
                align_depth_to_color=False,
            )
            try:
                camera.start()
                for _ in range(int(stream.get("warmup_frames", 20))):
                    camera.get_frames(timeout_ms=3000)
                intrinsics = camera.get_intrinsics("color")
            except Exception:
                try:
                    camera.stop()
                except Exception:
                    pass
                raise

            with self._lock:
                self._camera = camera
                self._active = name
                self._intrinsics = intrinsics
                self._frame = None
                self._frame_id = 0
                self._error = None
                self._stop_event.clear()
                self._thread = threading.Thread(target=self._capture_loop, name="d435-latest-frame", daemon=True)
                self._thread.start()
            return self.status()

    def _capture_loop(self) -> None:
        count = 0
        period_start = time.monotonic()
        while not self._stop_event.is_set():
            with self._lock:
                camera = self._camera
            if camera is None:
                return
            try:
                packet = camera.get_frames(timeout_ms=1500)
                frame = packet.get("color")
                if frame is None:
                    continue
                now = time.monotonic()
                count += 1
                elapsed = now - period_start
                with self._lock:
                    self._frame = frame
                    self._frame_id += 1
                    self._timestamp_ms = packet.get("timestamp_ms")
                    self._received_monotonic = now
                    if elapsed >= 1.0:
                        self._fps = count / elapsed
                        count = 0
                        period_start = now
            except Exception as exc:
                if not self._stop_event.is_set():
                    with self._lock:
                        self._error = f"取帧失败: {exc}"
                time.sleep(0.05)

    def latest_frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def mjpeg(self) -> Iterator[bytes]:
        stream = self.config["camera_stream"]
        width = int(stream.get("preview_width", 1120))
        quality = int(stream.get("preview_jpeg_quality", 78))
        delay = 1.0 / max(1, int(stream.get("preview_fps", 25)))
        last_id = -1
        while True:
            with self._lock:
                frame = None if self._frame is None else self._frame.copy()
                frame_id = self._frame_id
                active = self._active
            if active is None:
                return
            if frame is None or frame_id == last_id:
                time.sleep(0.01)
                continue
            last_id = frame_id
            if frame.shape[1] > width:
                scale = width / frame.shape[1]
                frame = cv2.resize(frame, (width, int(frame.shape[0] * scale)), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + encoded.tobytes() + b"\r\n"
            time.sleep(delay)

    def stop(self) -> dict[str, Any]:
        with self._switch_lock:
            self._stop_locked()
        return self.status()

    def _stop_locked(self) -> None:
        self._stop_event.set()
        with self._lock:
            camera = self._camera
            thread = self._thread
            self._camera = None
            self._thread = None
            self._active = None
            self._frame = None
            self._intrinsics = None
        if camera is not None:
            try:
                camera.stop()
            except Exception as exc:
                self._error = f"停止相机失败: {exc}"
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
