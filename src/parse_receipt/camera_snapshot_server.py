"""Camera HTTP bridge for remote receipt / perception services.

  GET /health
  GET /camera/snapshot?camera=head&type=color  -> one JPEG
  GET /camera/video?camera=head                 -> live MJPEG stream
  GET /                                         -> simple live preview page

Bind 0.0.0.0 so peers on the LAN can reach this host.
Use conda env hand_eye_calib or robot (needs pyrealsense2).
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Dict, Iterator, Tuple

import cv2
import uvicorn
from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import HTMLResponse, Response, StreamingResponse

DEFAULT_CAMERA_API = "/home/lh/robot_api/camera_api"
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8083
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_WARMUP = 20
DEFAULT_JPEG_QUALITY = 85
DEFAULT_PREVIEW_FPS = 15
DEFAULT_PREVIEW_WIDTH = 960

CAMERA_SERIALS: Dict[str, str] = {
    "head": os.getenv("CAMERA_SERIAL_HEAD", "344422070170"),
    "left": os.getenv("CAMERA_SERIAL_LEFT", "215322079194"),
    "right": os.getenv("CAMERA_SERIAL_RIGHT", "335522072306"),
}

_CAPTURE_LOCK = threading.Lock()


def _camera_api_path() -> str:
    return os.getenv("CAMERA_API_PATH", DEFAULT_CAMERA_API)


def _import_d435():
    camera_api = _camera_api_path()
    if camera_api not in sys.path:
        sys.path.insert(0, camera_api)
    from D435_rgb_depth import D435Camera, list_realsense_devices

    return D435Camera, list_realsense_devices


def _positive_int(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return default
    value = int(raw)
    if value <= 0:
        raise ValueError(f"{name} must be > 0")
    return value


def _resolve_camera(camera_name: str, image_type: str = "color"):
    if image_type != "color":
        raise HTTPException(
            status_code=400,
            detail=f"unsupported type={image_type!r}; only type=color is supported",
        )
    serial = CAMERA_SERIALS.get(camera_name)
    if not serial:
        raise HTTPException(
            status_code=400,
            detail=f"unknown camera={camera_name!r}; expected one of {sorted(CAMERA_SERIALS)}",
        )

    try:
        D435Camera, list_realsense_devices = _import_d435()
        devices = list_realsense_devices()
    except ModuleNotFoundError as exc:
        raise HTTPException(
            status_code=503,
            detail=(
                f"camera dependency missing: {exc.name}. "
                "Start with conda env hand_eye_calib or robot."
            ),
        ) from exc
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(
            status_code=503, detail=f"camera api import/list failed: {exc}"
        ) from exc

    serials = {str(item.get("serial", "")) for item in devices}
    if serial not in serials:
        raise HTTPException(
            status_code=503,
            detail=f"camera serial {serial} not found; present={sorted(s for s in serials if s)}",
        )

    width = _positive_int("CAMERA_WIDTH", DEFAULT_WIDTH)
    height = _positive_int("CAMERA_HEIGHT", DEFAULT_HEIGHT)
    fps = _positive_int("CAMERA_FPS", DEFAULT_FPS)
    cam = D435Camera(
        serial=serial,
        width=width,
        height=height,
        fps=fps,
        enable_color=True,
        enable_depth=False,
        align_depth_to_color=False,
    )
    return cam, serial


def capture_jpeg(camera_name: str, image_type: str) -> bytes:
    cam, _serial = _resolve_camera(camera_name, image_type)
    warmup = _positive_int("CAMERA_WARMUP_FRAMES", DEFAULT_WARMUP)
    quality = _positive_int("CAMERA_JPEG_QUALITY", DEFAULT_JPEG_QUALITY)

    if not _CAPTURE_LOCK.acquire(timeout=15.0):
        raise HTTPException(status_code=503, detail="camera busy")

    try:
        cam.start()
        color = None
        for _ in range(max(warmup, 1)):
            data = cam.get_frames()
            color = data.get("color")
            if color is None:
                time.sleep(0.02)
        if color is None:
            raise HTTPException(status_code=503, detail="failed to get color frame")
        ok, encoded = cv2.imencode(
            ".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not ok:
            raise HTTPException(status_code=500, detail="jpeg encode failed")
        return encoded.tobytes()
    except HTTPException:
        raise
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(
            status_code=503, detail=f"camera capture failed: {exc}"
        ) from exc
    finally:
        try:
            cam.stop()
        except Exception:  # noqa: BLE001
            pass
        _CAPTURE_LOCK.release()


def mjpeg_stream(camera_name: str) -> Iterator[bytes]:
    cam, serial = _resolve_camera(camera_name, "color")
    warmup = _positive_int("CAMERA_WARMUP_FRAMES", DEFAULT_WARMUP)
    quality = _positive_int("CAMERA_JPEG_QUALITY", DEFAULT_JPEG_QUALITY)
    preview_fps = _positive_int("CAMERA_PREVIEW_FPS", DEFAULT_PREVIEW_FPS)
    preview_width = _positive_int("CAMERA_PREVIEW_WIDTH", DEFAULT_PREVIEW_WIDTH)
    delay = 1.0 / max(1, preview_fps)

    if not _CAPTURE_LOCK.acquire(timeout=15.0):
        raise HTTPException(status_code=503, detail="camera busy")

    try:
        cam.start()
        print(f"live stream start, serial={serial}, camera={camera_name}")
        for _ in range(max(warmup, 1)):
            cam.get_frames()

        while True:
            data = cam.get_frames()
            color = data.get("color")
            if color is None:
                time.sleep(0.01)
                continue
            frame = color
            if frame.shape[1] > preview_width:
                scale = preview_width / frame.shape[1]
                frame = cv2.resize(
                    frame,
                    (preview_width, int(frame.shape[0] * scale)),
                    interpolation=cv2.INTER_AREA,
                )
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            if ok:
                chunk = encoded.tobytes()
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(chunk)).encode()
                    + b"\r\n\r\n"
                    + chunk
                    + b"\r\n"
                )
            time.sleep(delay)
    except GeneratorExit:
        return
    except Exception as exc:  # noqa: BLE001
        print(f"live stream error: {exc}")
        return
    finally:
        try:
            cam.stop()
        except Exception:  # noqa: BLE001
            pass
        _CAPTURE_LOCK.release()
        print(f"live stream stop, camera={camera_name}")


app = FastAPI(title="camera-http-bridge", version="0.2.0")


@app.get("/", response_class=HTMLResponse)
def index(camera: str = Query("head")) -> str:
    if camera not in CAMERA_SERIALS:
        camera = "head"
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>头部相机实时预览</title>
  <style>
    body {{ margin:0; font-family:sans-serif; background:#111; color:#eee; }}
    header {{ padding:12px 16px; background:#1c1c1c; }}
    main {{ padding:12px; }}
    img {{ max-width:100%; background:#000; border:1px solid #333; }}
    a {{ color:#8cf; }}
  </style>
</head>
<body>
  <header>
    <strong>相机实时预览</strong>
    （{camera}）
    · <a href="/camera/video?camera={camera}">原始视频流</a>
    · <a href="/camera/snapshot?camera={camera}&type=color">拍一张</a>
    · <a href="/health">health</a>
  </header>
  <main>
    <img src="/camera/video?camera={camera}" alt="live camera"/>
  </main>
</body>
</html>
"""


@app.get("/health")
def health() -> dict:
    return {
        "ok": True,
        "service": "camera-http-bridge",
        "cameras": sorted(CAMERA_SERIALS),
        "snapshot_path": "/camera/snapshot?camera=head&type=color",
        "video_path": "/camera/video?camera=head",
        "preview_page": "/",
        "camera_api": _camera_api_path(),
    }


@app.get("/camera/snapshot")
def camera_snapshot(
    camera: str = Query("head"),
    type: str = Query("color"),  # noqa: A002
) -> Response:
    jpeg = capture_jpeg(camera_name=camera, image_type=type)
    return Response(content=jpeg, media_type="image/jpeg")


@app.get("/camera/video")
def camera_video(camera: str = Query("head")) -> StreamingResponse:
    # Validate before starting the generator so clients get a proper HTTP error.
    _resolve_camera(camera, "color")
    return StreamingResponse(
        mjpeg_stream(camera),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
        },
    )


def main() -> None:
    host = os.getenv("CAMERA_SNAPSHOT_HOST", DEFAULT_HOST)
    port = _positive_int("CAMERA_SNAPSHOT_PORT", DEFAULT_PORT)
    print(f"live preview page : http://<robot-ip>:{port}/")
    print(
        f"snapshot          : http://<robot-ip>:{port}"
        f"/camera/snapshot?camera=head&type=color"
    )
    print(f"live video        : http://<robot-ip>:{port}/camera/video?camera=head")
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
