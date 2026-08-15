#!/usr/bin/env python3
"""Capture receipt images and preserve the exact image/result pair for review."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_CAMERA_URL = (
    "http://192.168.3.176:8083/camera/snapshot?camera=head&type=color"
)
DEFAULT_PERCEPTION_ROOT = Path("/home/kim/WRC/perception")
DEFAULT_OUTPUT_ROOT = Path("/home/kim/WRC/receipt_test_runs")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="保存头部相机小票图片，并用同一图片执行小票识别",
    )
    parser.add_argument("--count", type=int, default=1, help="采集识别次数，默认1")
    parser.add_argument(
        "--interval-seconds", type=float, default=1.0, help="多次测试间隔秒数"
    )
    parser.add_argument(
        "--camera-url",
        default=os.getenv("RECEIPT_CAMERA_URL", DEFAULT_CAMERA_URL),
    )
    parser.add_argument(
        "--perception-root",
        type=Path,
        default=DEFAULT_PERCEPTION_ROOT,
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    parser.add_argument("--camera-timeout", type=float, default=10.0)
    return parser.parse_args()


def capture_image(url: str, timeout: float) -> bytes:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request = urllib.request.Request(url, method="GET")
    with opener.open(request, timeout=timeout) as response:
        content_type = response.headers.get("Content-Type", "")
        payload = response.read()
    if not payload or not content_type.lower().startswith("image/"):
        raise RuntimeError(
            f"相机没有返回有效图片: content_type={content_type!r}, bytes={len(payload)}"
        )
    return payload


def response_to_dict(response: Any) -> dict[str, Any]:
    if hasattr(response, "model_dump"):
        value = response.model_dump()
    elif hasattr(response, "dict"):
        value = response.dict()
    elif isinstance(response, dict):
        value = response
    else:
        raise TypeError(f"无法序列化识别响应: {type(response).__name__}")
    if not isinstance(value, dict):
        raise TypeError("识别响应序列化后不是对象")
    return value


def load_local_parser(perception_root: Path):
    module_dir = perception_root / "parse_receipt"
    module_file = module_dir / "local_parse.py"
    if not module_file.is_file():
        raise FileNotFoundError(f"找不到本地小票解析模块: {module_file}")

    # 与4090当前本机服务保持一致；显式环境变量仍可覆盖这些默认值。
    os.environ.setdefault(
        "QWEN_BASE_URL", "http://127.0.0.1:8000/v1/chat/completions"
    )
    os.environ.setdefault("QWEN_MODEL", "qwen3-vl-4b")
    os.environ.setdefault("SKU_BASE_URL", "http://127.0.0.1:25540")
    sys.path.insert(0, str(module_dir))
    import local_parse  # type: ignore

    return local_parse


def append_summary(path: Path, record: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def main() -> int:
    args = parse_args()
    if args.count < 1:
        raise ValueError("--count必须大于等于1")
    if args.interval_seconds < 0:
        raise ValueError("--interval-seconds不能为负数")

    args.output_root.mkdir(parents=True, exist_ok=True)
    local_parse = load_local_parser(args.perception_root)
    summary_path = args.output_root / "summary.jsonl"

    for index in range(args.count):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        sample_dir = args.output_root / stamp
        sample_dir.mkdir(parents=False, exist_ok=False)
        image_path = sample_dir / "receipt.jpg"
        result_path = sample_dir / "result.json"
        metadata_path = sample_dir / "metadata.json"
        started = time.monotonic()
        success = False
        result: dict[str, Any] = {}
        error: str | None = None

        try:
            image_path.write_bytes(capture_image(args.camera_url, args.camera_timeout))
            # 识别刚刚保存的同一个文件，不触发第二次相机拍摄。
            response = local_parse.parse_image_file(image_path)
            result = response_to_dict(response)
            result_path.write_text(
                json.dumps(result, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            success = True
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            result_path.write_text(
                json.dumps({"error": error}, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

        record = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "sample_dir": str(sample_dir),
            "image": str(image_path),
            "result": str(result_path),
            "camera_url": args.camera_url,
            "success": success,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "product_names": result.get("product_names", []),
            "error": error,
        }
        metadata_path.write_text(
            json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        append_summary(summary_path, record)
        print(json.dumps(record, ensure_ascii=False, indent=2))

        if index + 1 < args.count and args.interval_seconds:
            time.sleep(args.interval_seconds)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
