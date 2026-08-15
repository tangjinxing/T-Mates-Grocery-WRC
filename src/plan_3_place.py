#!/usr/bin/env python3
"""步骤 3：后退点 → 放置点 mark_1 → 放置位姿 → /manipulation/release。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import plan


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "步骤 3：后退点（F→mark_6 / B→mark_7）→ mark_1 → "
            "DELIVERY_TABLE_PLACE_READY → release。默认 execute=true 真实放置。"
        )
    )
    parser.add_argument("--navigation-url", default=plan.DEFAULT_NAVIGATION_URL)
    parser.add_argument("--pose-url", default=plan.DEFAULT_POSE_URL)
    parser.add_argument("--nav-timeout", type=float, default=600.0)
    parser.add_argument("--pose-timeout", type=float, default=180.0)
    parser.add_argument("--release-timeout", type=float, default=300.0)
    parser.add_argument(
        "--nav-target",
        default="",
        help="刚才的货位，用来判断 F/B 后退点和左右手；默认读状态文件",
    )
    parser.add_argument(
        "--skip-pose",
        action="store_true",
        help="到 mark_1 后不调用 DELIVERY_TABLE_PLACE_READY / release",
    )
    parser.add_argument(
        "--skip-release",
        action="store_true",
        help="放置位姿后不调用 /manipulation/release",
    )
    parser.add_argument(
        "--no-execute-release",
        action="store_true",
        help="release 只检查不松爪；默认 execute=true 真实放置",
    )
    parser.add_argument(
        "--state",
        type=Path,
        default=plan.DEFAULT_STATE_PATH,
        help="步骤间状态文件，默认 %(default)s",
    )
    args = parser.parse_args()
    nav_target = args.nav_target.strip().upper() or None

    try:
        summary = plan.run_step3_place(
            navigation_url=args.navigation_url.rstrip("/"),
            pose_url=args.pose_url.rstrip("/"),
            nav_timeout=args.nav_timeout,
            pose_timeout=args.pose_timeout,
            skip_pose=args.skip_pose,
            nav_target=nav_target,
            state_path=args.state,
            skip_release=args.skip_release,
            execute_release_motion=not args.no_execute_release,
            release_timeout=args.release_timeout,
        )
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1

    plan.dump_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
