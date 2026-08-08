#!/usr/bin/env python3
"""左臂眼在手上独立网页入口：左手D435 + 左臂，固定端口8002。"""

from pathlib import Path
import sys

import uvicorn


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from eye_in_hand_web.web_calibration_app import build_app  # noqa: E402


if __name__ == "__main__":
    print("左臂眼在手上页面：http://机器人主机IP:8002")
    print("固定配置：左臂169.254.128.18，左手D435序列号215322079194")
    print("独立数据目录：../datasets/eye_in_hand_left")
    uvicorn.run(build_app(ROOT / "calibration_config.yaml", "eye_in_hand_left"), host="0.0.0.0", port=8002)
