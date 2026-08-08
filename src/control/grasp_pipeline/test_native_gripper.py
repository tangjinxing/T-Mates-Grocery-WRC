#!/usr/bin/env python3
"""左右臂因时原生夹爪的独立力控夹取/释放测试；不会移动机械臂。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


ARM_API_ROOT = Path("/home/lh/robot_api/arm_api_new")
ARM_IPS = {"left": "169.254.128.18", "right": "169.254.128.19"}
POLL_S = 0.2
ACTION_TIMEOUT_S = 15.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="左右臂因时原生夹爪力控测试（不移动机械臂）")
    parser.add_argument(
        "--arm", choices=("left", "right"), default="left",
        help="目标机械臂；默认left以兼容原有命令",
    )
    parser.add_argument("--speed", type=int, default=100, help="闭合/打开速度，首次测试建议100")
    parser.add_argument("--force", type=int, default=80, help="力控阈值，塑料瓶首次建议80")
    parser.add_argument("--open-only", action="store_true", help="只打开夹爪，用于释放或恢复")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 1 <= args.speed <= 300:
        raise ValueError("本测试将speed限制在1～300")
    if not 50 <= args.force <= 200:
        raise ValueError("本测试将force限制在50～200")


def require_healthy(state) -> None:
    if state.enable_state != 1:
        raise RuntimeError(f"夹爪未使能: {state}")
    if state.status != 1:
        raise RuntimeError(f"夹爪离线: {state}")
    if state.error != 0:
        raise RuntimeError(f"夹爪错误码={state.error}")


def wait_open(client, timeout: float = ACTION_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = client.get_gripper_state()
        require_healthy(last)
        print(
            f"打开监控: actpos={last.actpos}, force={last.current_force}, "
            f"mode={last.mode}",
            flush=True,
        )
        if last.actpos >= 950:
            return last
        time.sleep(POLL_S)
    raise TimeoutError(f"夹爪打开超过{timeout:.0f}s未到位，最后状态={last}")


def wait_force_grip(client, initial_position: int, timeout: float = ACTION_TIMEOUT_S):
    deadline = time.monotonic() + timeout
    previous_position = None
    movement_observed = False
    stable_samples = 0
    last = None

    while time.monotonic() < deadline:
        last = client.get_gripper_state()
        require_healthy(last)
        if abs(last.actpos - initial_position) >= 10:
            movement_observed = True
        if movement_observed and previous_position is not None and abs(last.actpos - previous_position) <= 3:
            stable_samples += 1
        else:
            stable_samples = 0
        previous_position = last.actpos
        print(
            f"闭合监控: actpos={last.actpos}, force={last.current_force}, mode={last.mode}, "
            f"已运动={movement_observed}, 稳定={stable_samples}/5",
            flush=True,
        )

        if stable_samples >= 5:
            if last.actpos <= 50:
                raise RuntimeError("夹爪接近完全闭合，可能没有夹到物品")
            if last.actpos >= 950:
                raise RuntimeError("夹爪仍接近全开，未确认夹取")
            if last.mode != 6:
                raise RuntimeError(
                    f"夹爪已停止但mode={last.mode}，"
                    "未确认为力控接触停止(mode=6)"
                )
            return last
        time.sleep(POLL_S)
    raise TimeoutError(f"力控闭合超过{timeout:.0f}s未稳定，最后状态={last}")


def open_gripper(client, speed: int):
    client.gripper_release(speed=speed, block=False, timeout=1)
    return wait_open(client)


def main() -> int:
    args = parse_args()
    validate_args(args)
    sys.path.insert(0, str(ARM_API_ROOT))
    from realman_arm_api_api2 import RealmanArmClient

    arm_ip = ARM_IPS[args.arm]
    client = RealmanArmClient(ip=arm_ip, model=args.arm, auto_connect=False)
    try:
        print(f"连接{args.arm}臂控制器: {arm_ip}")
        client.connect()
        initial = client.get_gripper_state()
        require_healthy(initial)
        print(f"初始夹爪状态: {initial}")
        client.configure_gripper_range(0, 1000)

        if args.open_only:
            confirmation = input("确认物品有支撑且打开不会掉落，输入 OPEN GRIPPER: ").strip()
            if confirmation != "OPEN GRIPPER":
                print("已取消，没有发送夹爪命令。")
                return 0
            result = open_gripper(client, args.speed)
            print(f"夹爪已打开: {result}")
            return 0

        print("\n请将物品放在两指之间并用外部平面支撑，手指离开夹爪运动范围。")
        confirmation = input(
            f"输入 CLOSE GRIPPER，以speed={args.speed}, force={args.force}执行力控闭合: "
        ).strip()
        if confirmation != "CLOSE GRIPPER":
            print("已取消，没有发送夹爪命令。")
            return 0

        before_close = client.get_gripper_state()
        require_healthy(before_close)
        client.gripper_pick_keep(speed=args.speed, force=args.force, block=False, timeout=1)
        result = wait_force_grip(client, before_close.actpos)
        print("\n夹取状态检查通过:")
        print(result)
        print("请人工轻推物品检查牢固度；不要抬升机械臂。")

        confirmation = input("检查完成且物品有支撑后，输入 OPEN GRIPPER 释放: ").strip()
        if confirmation != "OPEN GRIPPER":
            print("未打开夹爪；夹爪保持当前状态。可稍后使用 --open-only 释放。")
            return 0
        opened = open_gripper(client, args.speed)
        print(f"夹爪已重新打开: {opened}")
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n用户中止。夹爪不会自动打开，以免无支撑物品掉落。")
        print("确认物品有支撑后，使用 --open-only 释放。")
        raise SystemExit(130)
    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        print("若夹爪仍闭合，请确认物品有支撑后使用 --open-only。", file=sys.stderr)
        raise SystemExit(1)
