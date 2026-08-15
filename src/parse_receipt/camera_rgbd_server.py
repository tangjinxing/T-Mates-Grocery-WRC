#!/usr/bin/env python3
"""RGB-D HTTP 相机桥，默认 0.0.0.0:8085。

一次拍摄同时返回对齐的彩色图、uint16 深度图和 camera.json。
8083 仍只提供小票用的彩色 JPEG，不要混用。

  GET /health
  GET /camera/rgbd?camera=head|left|right            JSON（含 base64 图）
  GET /camera/rgbd?camera=head&format=zip            zip: rgb.png + depth.png + camera.json
  GET /camera/snapshot?camera=head&type=color        JPEG
  GET /camera/snapshot?camera=head&type=depth        16-bit PNG
"""

from __future__ import annotations

import base64
import io
import json
import os
import sys
import threading
import zipfile
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import Response

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from rgbd_capture import CAMERA_SERIALS, RgbdFrame, capture_rgbd

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8085
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_WARMUP = 20

_CAPTURE_LOCK = threading.Lock()
app = FastAPI(title="camera-rgbd-bridge", version="0.1.0")


def _positive_int(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return default
    value = int(raw)
    if value <= 0:
        raise ValueError(f"{name} must be > 0")
    return value


def _grab(camera: str) -> RgbdFrame:
    if camera not in CAMERA_SERIALS:
        raise HTTPException(
            status_code=400,
            detail=f"unknown camera={camera!r}; expected {sorted(CAMERA_SERIALS)}",
        )
    if not _CAPTURE_LOCK.acquire(timeout=15.0):
        raise HTTPException(status_code=503, detail="camera busy")
    try:
        return capture_rgbd(
            camera,
            width=_positive_int("CAMERA_WIDTH", DEFAULT_WIDTH),
            height=_positive_int("CAMERA_HEIGHT", DEFAULT_HEIGHT),
            fps=_positive_int("CAMERA_FPS", DEFAULT_FPS),
            warmup=_positive_int("CAMERA_WARMUP_FRAMES", DEFAULT_WARMUP),
        )
    except HTTPException:
        raise
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=503, detail=f"RGB-D capture failed: {exc}") from exc
    finally:
        _CAPTURE_LOCK.release()


def _depth_headers(frame: RgbdFrame) -> dict[str, str]:
    meta = frame.camera_json()
    return {
        "X-Camera": frame.camera,
        "X-Camera-Serial": frame.serial,
        "X-Image-Width": str(frame.width),
        "X-Image-Height": str(frame.height),
        "X-Depth-Scale": str(meta["depth_scale"]),
        "X-Depth-Scale-M": str(meta["depth_scale_m"]),
        "X-Cam-K": json.dumps(meta["cam_K"]),
        "X-Aligned-To-Color": "true",
    }


@app.get("/health")
def health() -> dict:
    return {
        "ok": True,
        "service": "camera-rgbd-bridge",
        "port": _positive_int("CAMERA_RGBD_PORT", DEFAULT_PORT),
        "cameras": sorted(CAMERA_SERIALS),
        "rgbd_path": "/camera/rgbd?camera=head",
        "color_path": "/camera/snapshot?camera=head&type=color",
        "depth_path": "/camera/snapshot?camera=head&type=depth",
        "note": "depth.png is uint16; depth_mm = raw * depth_scale (BOP, mm/unit)",
    }


@app.get("/camera/rgbd")
def camera_rgbd(
    camera: str = Query("head"),
    format: str = Query("json"),  # noqa: A002
) -> Response:
    frame = _grab(camera)
    meta = frame.camera_json()
    rgb_png = frame.rgb_png()
    depth_png = frame.depth_png()

    if format == "zip":
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("rgb.png", rgb_png)
            archive.writestr("depth.png", depth_png)
            archive.writestr(
                "camera.json",
                json.dumps(meta, ensure_ascii=False, indent=2) + "\n",
            )
        return Response(
            content=buf.getvalue(),
            media_type="application/zip",
            headers={
                "Content-Disposition": f'attachment; filename="{camera}_rgbd.zip"',
                **_depth_headers(frame),
            },
        )

    if format != "json":
        raise HTTPException(status_code=400, detail="format must be json or zip")

    payload = {
        **meta,
        "rgb_png_b64": base64.b64encode(rgb_png).decode("ascii"),
        "depth_png_b64": base64.b64encode(depth_png).decode("ascii"),
    }
    return Response(
        content=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        media_type="application/json",
        headers=_depth_headers(frame),
    )


@app.get("/camera/snapshot")
def camera_snapshot(
    camera: str = Query("head"),
    type: str = Query("color"),  # noqa: A002
) -> Response:
    if type not in {"color", "depth"}:
        raise HTTPException(
            status_code=400,
            detail="type must be color (jpeg) or depth (16-bit png)",
        )
    frame = _grab(camera)
    headers = _depth_headers(frame)
    if type == "color":
        return Response(
            content=frame.rgb_jpeg(),
            media_type="image/jpeg",
            headers=headers,
        )
    return Response(
        content=frame.depth_png(),
        media_type="image/png",
        headers=headers,
    )


def main() -> None:
    host = os.getenv("CAMERA_RGBD_HOST", DEFAULT_HOST)
    port = _positive_int("CAMERA_RGBD_PORT", DEFAULT_PORT)
    print(f"RGB-D health : http://<robot-ip>:{port}/health")
    print(f"RGB-D json   : http://<robot-ip>:{port}/camera/rgbd?camera=head")
    print(f"RGB-D zip    : http://<robot-ip>:{port}/camera/rgbd?camera=head&format=zip")
    print(
        f"depth png    : http://<robot-ip>:{port}"
        "/camera/snapshot?camera=head&type=depth"
    )
    uvicorn.run(app, host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
