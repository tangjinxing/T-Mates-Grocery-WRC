#!/usr/bin/env python3
"""左臂动态眼在手外独立网页入口，固定使用端口8001与左臂配置。"""

from pathlib import Path
import sys

import uvicorn


ROOT = Path(__file__).resolve().parent
WEB_DIR = ROOT / "dynamic_handeye_web"
sys.path.insert(0, str(WEB_DIR))

from web_calibration_app import build_app  # noqa: E402


PROFILE = "eye_to_hand_left"
HOST = "0.0.0.0"
PORT = 8001


if __name__ == "__main__":
    app = build_app(ROOT / "calibration_config.yaml", PROFILE)
    print("左臂动态眼在手外页面：http://机器人主机IP:8001")
    print("固定配置：左臂169.254.128.18，头部D435序列号344422070170")
    print("独立数据目录：../datasets/dynamic_eye_to_hand_left")
    uvicorn.run(app, host=HOST, port=PORT)
