#!/usr/bin/env python3
"""步骤 1：小票位 → RECEIPT_VIEW → 识别 → 货位。停在货位，不抓取、不放置。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import plan


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "步骤 1：小票位 → RECEIPT_VIEW → 识别 → 只去第一件货位。"
            "第二件用 --next。"
        )
    )
    parser.add_argument("--perception-url", default=plan.DEFAULT_PERCEPTION_URL)
    parser.add_argument("--sku-url", default=plan.DEFAULT_SKU_URL)
    parser.add_argument("--navigation-url", default=plan.DEFAULT_NAVIGATION_URL)
    parser.add_argument("--pose-url", default=plan.DEFAULT_POSE_URL)
    parser.add_argument("--parse-timeout", type=float, default=180.0)
    parser.add_argument("--sku-timeout", type=float, default=5.0)
    parser.add_argument("--nav-timeout", type=float, default=600.0)
    parser.add_argument("--pose-timeout", type=float, default=180.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument(
        "--skip-receipt-station",
        action="store_true",
        help="已经在小票位时跳过 mark_0",
    )
    parser.add_argument(
        "--skip-receipt",
        action="store_true",
        help="跳过小票/SKU；此时必须给 --nav-target",
    )
    parser.add_argument(
        "--skip-pose",
        action="store_true",
        help="到达小票位后不去 RECEIPT_VIEW",
    )
    parser.add_argument(
        "--nav-target",
        default="",
        help="覆盖货位；默认用第一件 SKU 货位",
    )
    parser.add_argument(
        "--product-name",
        default="",
        help="只跑这一件商品；默认小票第一件",
    )
    parser.add_argument(
        "--next",
        dest="next_item",
        action="store_true",
        help="不重新识别，导航到状态里的下一件商品货位",
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
    if args.next_item and args.skip_receipt:
        parser.error("--next 与 --skip-receipt 不要一起用")
    if args.skip_receipt and not nav_target:
        parser.error("--skip-receipt 需要 --nav-target")

    try:
        summary = plan.run_step1_receipt_to_shelf(
            perception_url=args.perception_url.rstrip("/"),
            sku_url=args.sku_url.rstrip("/"),
            navigation_url=args.navigation_url.rstrip("/"),
            pose_url=args.pose_url.rstrip("/"),
            parse_timeout=args.parse_timeout,
            sku_timeout=args.sku_timeout,
            nav_timeout=args.nav_timeout,
            pose_timeout=args.pose_timeout,
            skip_receipt=args.skip_receipt,
            go_receipt_station=not args.skip_receipt_station and not args.next_item,
            skip_pose=args.skip_pose,
            nav_target=nav_target,
            settle_seconds=args.settle_seconds,
            state_path=args.state,
            product_name=product_name,
            next_item=args.next_item,
        )
    except Exception as exc:
        print(f"失败: {exc}", file=sys.stderr)
        return 1

    plan.dump_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
