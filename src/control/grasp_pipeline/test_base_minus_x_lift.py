#!/usr/bin/env python3
"""仅验证机械臂基座系 -X 方向的实际运动方向。

脚本读取当前末端6D位姿，保持姿态、Y和Z不变，仅将基座系X坐标减少指定距离。
脚本只执行一次单向MoveL，不自动返回，便于现场观察机械臂实际运动方向。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
ARM_API_ROOT = Path("/home/lh/robot_api/arm_api_new")
ARM_IPS = {"left": "169.254.128.18", "right": "169.254.128.19"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="验证基座系-X方向的低速单向移动")
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument(
        "--distance-mm",
        type=float,
        default=30.0,
        help="沿基座系-X移动距离，范围30～40 mm，默认30 mm",
    )
    parser.add_argument("--speed", type=float, default=2.0, help="MoveL速度百分比，默认2%%")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 30.0 <= args.distance_mm <= 40.0:
        raise ValueError("测试距离必须在30～40 mm范围内")
    if not 0.5 <= args.speed <= 5.0:
        raise ValueError("测试速度必须在0.5～5%范围内")

    sys.path.insert(0, str(ROOT))
    from run_pose_target import execute_movel_monitored, inverse_kinematics, nonzero_robot_errors

    sys.path.insert(0, str(ARM_API_ROOT))
    from realman_arm_api_api2 import RealmanArmClient

    client = RealmanArmClient(ip=ARM_IPS[args.arm], model=args.arm, auto_connect=False)
    try:
        print(f"连接{args.arm}臂: {ARM_IPS[args.arm]}")
        client.connect()
        state = client.get_state()
        robot_errors = nonzero_robot_errors(state.err)
        if robot_errors:
            raise RuntimeError("机械臂当前存在错误: " + ", ".join(robot_errors))

        current_pose = [float(value) for value in state.pose.as_list()]
        target_pose = current_pose.copy()
        target_pose[0] -= args.distance_mm / 1000.0

        current_joints = [float(value) for value in state.joints]
        target_joints = inverse_kinematics(client, current_joints, target_pose)
        joint_deltas = [
            abs(float(target) - float(current))
            for target, current in zip(target_joints, current_joints)
        ]

        print("\n========== 基座-X方向测试 ==========")
        print("当前末端6D:", current_pose)
        print("目标末端6D:", target_pose)
        print(f"计划变化: base X -{args.distance_mm:.1f} mm，Y/Z和姿态保持不变")
        print("当前关节角(deg):", current_joints)
        print("目标逆解(deg):", target_joints)
        print("最大关节变化(deg):", max(joint_deltas, default=0.0))
        print("注意：脚本只执行这一次单向MoveL，完成后不会自动返回。")

        confirmation = input(
            f"确认现场空间安全，并输入 MOVE {args.arm.upper()} MINUS X {args.distance_mm:g}MM: "
        ).strip()
        expected = f"MOVE {args.arm.upper()} MINUS X {args.distance_mm:g}MM"
        if confirmation != expected:
            print("确认词不匹配，未发送任何运动命令。")
            return 0

        actual_pose = execute_movel_monitored(
            client,
            target_pose,
            speed_percent=args.speed,
            timeout_s=20.0,
        )
        print("\nMoveL完成，请现场观察机械臂实际运动方向。")
        print("实际末端6D:", actual_pose)
        print(
            "实际位移(mm):",
            [
                (float(actual_pose[index]) - current_pose[index]) * 1000.0
                for index in range(3)
            ],
        )
        print("脚本不会自动返回；请确认方向后使用示教或单独的安全回位流程返回。")
        return 0
    finally:
        client.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n用户中止。")
        raise SystemExit(130)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
