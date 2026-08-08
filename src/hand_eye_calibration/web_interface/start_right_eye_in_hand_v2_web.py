#!/usr/bin/env python3
"""右臂眼在手上v2独立网页入口：右手D435 + 右臂，固定端口8003。"""

from pathlib import Path
import sys

import uvicorn


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from eye_in_hand_web.web_calibration_app import build_app  # noqa: E402


if __name__ == "__main__":
    print("右臂眼在手上v2页面：http://机器人主机IP:8003")
    print("固定配置：右臂169.254.128.19，右手D435序列号335522072306")
    print("独立数据目录：../datasets/eye_in_hand_right_v2")
    uvicorn.run(
        build_app(ROOT / "calibration_config.yaml", "eye_in_hand_right_v2"),
        host="0.0.0.0",
        port=8003,
    )
