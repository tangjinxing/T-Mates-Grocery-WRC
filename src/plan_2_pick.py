#!/usr/bin/env python3
"""步骤 2：拍照 → locate → pick_pose → grasp。假定已经在货位。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import plan


def main() -> int:
    parser = argparse.ArgumentParser(
        description="步骤 2：拍照 → locate → pick_pose → grasp（已在货位）"
    )
    parser.add_argument("--perception-url", default=plan.DEFAULT_PERCEPTION_URL)
    parser.add_argument("--rgbd-url", default=plan.DEFAULT_RGBD_URL)
    parser.add_argument("--pose-url", default=plan.DEFAULT_POSE_URL)
    parser.add_argument("--pick-pose-url", default=plan.DEFAULT_PICK_POSE_URL)
    parser.add_argument("--pose-timeout", type=float, default=180.0)
    parser.add_argument("--rgbd-timeout", type=float, default=30.0)
    parser.add_argument("--locate-timeout", type=float, default=180.0)
    parser.add_argument("--pick-pose-timeout", type=float, default=180.0)
    parser.add_argument("--grasp-timeout", type=float, default=300.0)
    parser.add_argument(
        "--snapshot-dir",
        type=Path,
        default=plan.DEFAULT_SNAPSHOT_DIR,
    )
    parser.add_argument(
        "--nav-target",
        default="",
        help="货位；默认读步骤 1 写入的状态文件",
    )
    parser.add_argument(
        "--product-name",
        default="",
        help="locate 商品名；默认用步骤 1 的 SKU 结果",
    )
    parser.add_argument(
        "--task-type",
        default=plan.DEFAULT_TASK_TYPE,
        choices=["SORTING", "SHORTAGE", "MISPLACED"],
    )
    parser.add_argument("--skip-pose", action="store_true", help="不走到层拍照位姿")
    parser.add_argument("--skip-capture", action="store_true", help="不拍 RGB-D")
    parser.add_argument("--skip-locate", action="store_true", help="不调用 locate / pick_pose")
    parser.add_argument("--skip-grasp", action="store_true", help="不调用 grasp")
    parser.add_argument(
        "--execute-grasp",
        dest="execute_grasp",
        action="store_true",
        default=True,
        help="grasp 时 execute=true，会真实抓取（默认）",
    )
    parser.add_argument(
        "--no-execute-grasp",
        dest="execute_grasp",
        action="store_false",
        help="grasp 时 execute=false，只规划不动臂",
    )
    parser.add_argument(
        "--state",
        type=Path,
        default=plan.DEFAULT_STATE_PATH,
        help="步骤间状态文件，默认 %(default)s",
    )
    args = parser.parse_args()
    nav_target = args.nav_target.strip().upper() or None
    product_name = args.product_name.strip() or None

    try:
        summary = plan.run_step2_pick(
            perception_url=args.perception_url.rstrip("/"),
            rgbd_url=args.rgbd_url.rstrip("/"),
            pose_url=args.pose_url.rstrip("/"),
            pick_pose_url=args.pick_pose_url.rstrip("/"),
            pose_timeout=args.pose_timeout,
            rgbd_timeout=args.rgbd_timeout,
            locate_timeout=args.locate_timeout,
            pick_pose_timeout=args.pick_pose_timeout,
            grasp_timeout=args.grasp_timeout,
            skip_pose=args.skip_pose,
            capture=not args.skip_capture,
            locate=not args.skip_locate,
            grasp=not args.skip_grasp,
            execute_grasp_motion=args.execute_grasp,
            nav_target=nav_target,
            product_name=product_name,
            task_type=args.task_type,
            snapshot_dir=args.snapshot_dir,
            state_path=args.state,
        )
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1

    plan.dump_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
