#!/usr/bin/env python3
"""左右臂固定点放置测试。

从 capability_poses.yaml 读取 DELIVERY_TABLE_PLACE_READY、对应机械臂的
TRANSITION 与 FINAL，执行 READY -> TRANSITION -> FINAL -> 松爪 ->
TRANSITION -> READY。脚本不移动躯干；启动前要求躯干已经位于放置高度，
目标机械臂位于 READY 且夹爪正在夹持物品。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parent
CAPABILITY_POSE_PATH = Path("/home/lh/WRC/src/capability_api/config/capability_poses.yaml")
ARM_API_ROOT = Path("/home/lh/robot_api/arm_api_new")
ARM_CONFIG = {
    "left": {"ip": "169.254.128.18", "pose_key": "left_arm"},
    "right": {"ip": "169.254.128.19", "pose_key": "right_arm"},
}

MIN_LIFT_HEIGHT_MM = 100
MAX_LIFT_HEIGHT_MM = 1350
LIFT_ARRIVAL_TOLERANCE_MM = 10
ARM_POSITION_TOLERANCE_M = 0.010
ARM_ORIENTATION_TOLERANCE_DEG = 3.0
START_POSITION_TOLERANCE_M = 0.020
START_ORIENTATION_TOLERANCE_DEG = 5.0
MAX_MOVE_M = 0.600
MAX_JOINT_DELTA_DEG = 90.0
POSITION_LIMITS_M = {
    "x": (-1.0, 1.0),
    "y": (-1.0, 1.0),
    "z": (-1.0, 1.5),
}
POLL_S = 0.2
STABLE_SAMPLES = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="左右臂固定点放置与安全回位测试")
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--plan-only", action="store_true", help="只做状态和完整路径逆解检查，不发送运动或夹爪命令")
    parser.add_argument("--arm-speed", type=float, default=5.0, help="MoveJ_P速度百分比，1～10")
    parser.add_argument("--lift-speed", type=int, default=10, help="升降柱速度百分比，1～20")
    parser.add_argument("--arm-timeout", type=float, default=60.0)
    parser.add_argument("--lift-timeout", type=float, default=90.0)
    return parser.parse_args()


def load_capability_poses() -> dict[str, Any]:
    if not CAPABILITY_POSE_PATH.is_file():
        raise FileNotFoundError(f"能力位姿文件不存在: {CAPABILITY_POSE_PATH}")
    data = yaml.safe_load(CAPABILITY_POSE_PATH.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("poses"), dict):
        raise ValueError(f"能力位姿文件结构异常: {CAPABILITY_POSE_PATH}")
    return data["poses"]


def finite_pose(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{label}必须是长度为6的列表")
    pose = [float(item) for item in value]
    if not all(math.isfinite(item) for item in pose):
        raise ValueError(f"{label}包含非有限数值")
    return pose


def load_targets(arm: str) -> tuple[list[float], list[float], int, list[float], str, str]:
    poses = load_capability_poses()
    side = arm.upper()
    arm_key = ARM_CONFIG[arm]["pose_key"]
    ready_name = "DELIVERY_TABLE_PLACE_READY"
    transition_name = f"DELIVERY_TABLE_PLACE_TRANSITION_{side}"
    final_name = f"DELIVERY_TABLE_PLACE_FINAL_{side}"
    for name in (ready_name, transition_name, final_name):
        if name not in poses:
            raise ValueError(f"能力位姿不存在: {name}")
    ready, transition, final = poses[ready_name], poses[transition_name], poses[final_name]
    for name, value in ((transition_name, transition), (final_name, final)):
        if value.get("status") not in {"candidate", "ready"}:
            raise ValueError(f"{name}状态不是candidate/ready: {value.get('status')}")
    ready_pose = finite_pose(ready.get(arm_key, {}).get("pose_6d"), f"{ready_name}.{arm_key}.pose_6d")
    transition_pose = finite_pose(
        transition.get(arm_key, {}).get("pose_6d"), f"{transition_name}.{arm_key}.pose_6d"
    )
    final_pose = finite_pose(final.get(arm_key, {}).get("pose_6d"), f"{final_name}.{arm_key}.pose_6d")
    heights = [ready.get("torso", {}).get("height_mm"), transition.get("torso", {}).get("height_mm"), final.get("torso", {}).get("height_mm")]
    if any(value is None for value in heights) or len({int(value) for value in heights}) != 1:
        raise ValueError(f"READY/TRANSITION/FINAL躯干高度不一致: {heights}")
    place_height = int(heights[0])
    if not MIN_LIFT_HEIGHT_MM <= place_height <= MAX_LIFT_HEIGHT_MM:
        raise ValueError(f"放置躯干高度{place_height}mm超出允许范围")
    return transition_pose, final_pose, place_height, ready_pose, transition_name, final_name


def nonzero_robot_errors(value: Any) -> list[str]:
    found: list[str] = []

    def visit(item: Any, path: str = "err") -> None:
        if isinstance(item, dict):
            for key, child in item.items():
                if key in {"err_len", "length", "count"}:
                    continue
                visit(child, f"{path}.{key}")
        elif isinstance(item, (list, tuple)):
            for index, child in enumerate(item):
                visit(child, f"{path}[{index}]")
        elif isinstance(item, (int, float)) and item != 0:
            found.append(f"{path}={item}")
        elif isinstance(item, str):
            text = item.strip()
            if text and text not in {"0", "0x0", "0X0"}:
                found.append(f"{path}={text}")

    visit(value)
    return found


def orientation_error_deg(actual: list[float], target: list[float]) -> float:
    actual_rotation = Rotation.from_euler("xyz", actual[3:], degrees=False)
    target_rotation = Rotation.from_euler("xyz", target[3:], degrees=False)
    return float(np.degrees((target_rotation.inv() * actual_rotation).magnitude()))


def pose_errors(actual: list[float], target: list[float]) -> tuple[float, float]:
    position_error = float(np.linalg.norm(np.asarray(actual[:3]) - np.asarray(target[:3])))
    return position_error, orientation_error_deg(actual, target)


def validate_pose_limits(label: str, pose: list[float], errors: list[str]) -> None:
    for index, axis in enumerate(("x", "y", "z")):
        low, high = POSITION_LIMITS_M[axis]
        if not low <= pose[index] <= high:
            errors.append(f"{label} {axis}={pose[index]:.3f}m超出[{low}, {high}]m")


def inverse_kinematics(client: Any, current_joints: list[float], target_pose: list[float]) -> list[float]:
    from Robotic_Arm.rm_ctypes_wrap import rm_inverse_kinematics_params_t

    q_in = ([float(value) for value in current_joints] + [0.0] * 7)[:7]
    params = rm_inverse_kinematics_params_t(q_in=q_in, q_pose=target_pose, flag=1)
    raw = client.raw_call("rm_algo_inverse_kinematics", params, check=False)
    if not isinstance(raw, tuple) or len(raw) != 2:
        raise RuntimeError(f"逆运动学返回格式异常: {raw!r}")
    code, joints = raw
    if int(code) != 0:
        raise RuntimeError(f"目标不可达，逆运动学错误码={code}")
    return [float(value) for value in joints]


def check_arm_motion(client: Any, label: str, target_pose: list[float]) -> dict[str, Any]:
    state = client.get_state()
    errors = nonzero_robot_errors(state.err)
    actual_pose = state.pose.as_list()
    validate_pose_limits(label, target_pose, errors)
    distance = float(np.linalg.norm(np.asarray(target_pose[:3]) - np.asarray(actual_pose[:3])))
    if not 0.0 < distance <= MAX_MOVE_M:
        errors.append(f"{label}位移{distance:.3f}m不在(0, {MAX_MOVE_M:.3f}]m")
    target_joints = None
    deltas = None
    if not errors:
        try:
            target_joints = inverse_kinematics(client, state.joints, target_pose)
            deltas = [abs(float(target) - float(current)) for current, target in zip(state.joints, target_joints)]
            if deltas and max(deltas) > MAX_JOINT_DELTA_DEG:
                errors.append(
                    f"{label}逆解单关节最大变化{max(deltas):.1f}°，超过{MAX_JOINT_DELTA_DEG:.1f}°"
                )
        except Exception as exc:
            errors.append(str(exc))
    print(f"\n========== {label}运动前安全检查 ==========")
    print(f"当前末端6D: {actual_pose}")
    print(f"目标末端6D: {target_pose}")
    print(f"直线位置差(仅用于限幅，不代表MoveJ_P轨迹): {distance:.3f}m")
    print(f"目标逆解(deg): {target_joints}")
    print(f"关节变化绝对值: {deltas}")
    if errors:
        print("[REJECTED]")
        for error in errors:
            print(f"  - {error}")
        raise RuntimeError(f"{label}运动前安全检查未通过")
    print("[PASS] 数值、工作空间、错误状态、距离和逆运动学检查通过")
    print("[WARN] 程序没有环境模型，无法自动判断MoveJ_P中间轨迹是否碰撞")
    return {"state": state, "target_joints": target_joints, "joint_deltas": deltas}


def require_pose(client: Any, label: str, target: list[float], position_tol: float, orientation_tol: float) -> list[float]:
    state = client.get_state()
    robot_errors = nonzero_robot_errors(state.err)
    if robot_errors:
        raise RuntimeError(f"{label}机械臂错误: {', '.join(robot_errors)}")
    actual = state.pose.as_list()
    position_error, rotation_error = pose_errors(actual, target)
    print(f"{label}: 位置误差={position_error * 1000.0:.1f}mm，姿态误差={rotation_error:.2f}°")
    if position_error > position_tol or rotation_error > orientation_tol:
        raise RuntimeError(
            f"{label}不满足要求：位置容差{position_tol * 1000.0:.0f}mm，姿态容差{orientation_tol:.1f}°"
        )
    return actual


def check_lift(client: Any, expected_height: int | None = None) -> int:
    status = client.get_lift_status()
    if status.err != 0:
        raise RuntimeError(f"升降柱错误码={status.err}，原始状态={status.raw}")
    if not MIN_LIFT_HEIGHT_MM <= status.height <= MAX_LIFT_HEIGHT_MM:
        raise RuntimeError(f"升降柱实际高度{status.height}mm超出允许范围")
    if expected_height is not None and abs(status.height - expected_height) > LIFT_ARRIVAL_TOLERANCE_MM:
        raise RuntimeError(
            f"升降柱实际高度{status.height}mm不在目标{expected_height}mm的"
            f"±{LIFT_ARRIVAL_TOLERANCE_MM}mm内"
        )
    return status.height


def check_gripper_holding(client: Any) -> Any:
    state = client.get_gripper_state()
    if state.enable_state != 1 or state.status != 1 or state.error != 0:
        raise RuntimeError(f"夹爪未使能或存在错误: {state}")
    if state.actpos <= 50:
        raise RuntimeError(f"夹爪接近全闭(actpos={state.actpos})，未确认夹到物体")
    if state.mode != 6:
        raise RuntimeError(
            f"夹爪当前mode={state.mode}，未确认为力控接触保持(mode=6): {state}"
        )
    print(
        f"夹爪夹持状态: actpos={state.actpos}, force={state.current_force}, "
        f"mode={state.mode}, error={state.error}"
    )
    return state


def execute_movej_p_monitored(
    client: Any,
    target_pose: list[float],
    speed: float,
    timeout_s: float,
    label: str,
) -> list[float]:
    deadline = time.monotonic() + timeout_s
    stable = 0
    last_report = 0.0
    command_sent = False
    try:
        client.movej_p(target_pose, v=speed, r=0, trajectory_connect=0, block=False)
        command_sent = True
        while True:
            now = time.monotonic()
            state = client.get_state()
            errors = nonzero_robot_errors(state.err)
            if errors:
                raise RuntimeError(f"{label}期间机械臂错误: {', '.join(errors)}")
            actual = state.pose.as_list()
            position_error, rotation_error = pose_errors(actual, target_pose)
            if now - last_report >= 0.5:
                print(
                    f"{label}监控: 位置误差={position_error * 1000.0:.1f}mm，"
                    f"姿态误差={rotation_error:.2f}°，稳定={stable}/{STABLE_SAMPLES}"
                )
                last_report = now
            if position_error <= ARM_POSITION_TOLERANCE_M and rotation_error <= ARM_ORIENTATION_TOLERANCE_DEG:
                stable += 1
                if stable >= STABLE_SAMPLES:
                    return actual
            else:
                stable = 0
            if now >= deadline:
                raise TimeoutError(
                    f"{label}超过{timeout_s:.0f}s仍未到位，位置误差={position_error * 1000.0:.1f}mm，"
                    f"姿态误差={rotation_error:.2f}°"
                )
            time.sleep(POLL_S)
    except BaseException:
        if command_sent:
            try:
                client.move_stop()
            except Exception as stop_exc:
                print(f"[WARN] 停止机械臂失败: {stop_exc}", file=sys.stderr)
        raise


def execute_lift_monitored(client: Any, target: int, speed: int, timeout_s: float, label: str) -> int:
    if not MIN_LIFT_HEIGHT_MM <= target <= MAX_LIFT_HEIGHT_MM:
        raise ValueError(f"升降柱目标{target}mm超出允许范围")
    current = check_lift(client)
    print(f"\n========== {label}运动前安全检查 ==========")
    print(f"升降柱当前={current}mm，目标={target}mm，速度={speed}%")
    if abs(current - target) <= LIFT_ARRIVAL_TOLERANCE_MM:
        print("升降柱已在目标容差内，无需发送运动命令")
        return current
    deadline = time.monotonic() + timeout_s
    stable = 0
    command_sent = False
    try:
        client.control_lift("to", speed=speed, height=target, block=False)
        command_sent = True
        while True:
            status = client.get_lift_status()
            if status.err != 0:
                raise RuntimeError(f"{label}期间升降柱错误码={status.err}")
            error = abs(status.height - target)
            print(f"{label}监控: 实际={status.height}mm，误差={error}mm")
            if error <= LIFT_ARRIVAL_TOLERANCE_MM:
                stable += 1
                if stable >= STABLE_SAMPLES:
                    return status.height
            else:
                stable = 0
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{label}超过{timeout_s:.0f}s仍未到位，最后误差={error}mm")
            time.sleep(0.5)
    except BaseException:
        if command_sent:
            try:
                client.control_lift("stop")
            except Exception as stop_exc:
                print(f"[WARN] 停止升降柱失败: {stop_exc}", file=sys.stderr)
        raise


def release_gripper_monitored(client: Any, timeout_s: float = 10.0) -> Any:
    client.gripper_release(speed=100, block=False, timeout=1)
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        last = client.get_gripper_state()
        if last.error != 0:
            raise RuntimeError(f"松开期间夹爪错误码={last.error}")
        print(f"夹爪松开监控: actpos={last.actpos}, error={last.error}")
        if last.enable_state == 1 and last.actpos >= 990:
            return last
        time.sleep(POLL_S)
    raise TimeoutError(f"夹爪未在{timeout_s:.0f}s内完全打开，最后状态={last}")


def confirm(exact_text: str, prompt: str) -> bool:
    answer = input(f"{prompt}\n输入 {exact_text}: ").strip()
    if answer != exact_text:
        print("确认词不匹配，流程停止；没有发送本阶段运动命令。")
        return False
    return True


def preflight_chain(client: Any, ready_pose: list[float], transition_pose: list[float], final_pose: list[float]) -> None:
    state = client.get_state()
    robot_errors = nonzero_robot_errors(state.err)
    if robot_errors:
        raise RuntimeError(f"机械臂存在错误: {', '.join(robot_errors)}")
    stages = [
        ("READY", ready_pose),
        ("TRANSITION", transition_pose),
        ("FINAL", final_pose),
        ("TRANSITION_RETURN", transition_pose),
        ("READY_RETURN", ready_pose),
    ]
    previous = [float(value) for value in state.joints]
    solved: dict[str, list[float]] = {}
    min_abs_j5 = float("inf")
    print("\n========== 完整放置链只读逆解 ==========")
    for label, pose in stages:
        errors: list[str] = []
        validate_pose_limits(label, pose, errors)
        if errors:
            raise RuntimeError("；".join(errors))
        current = inverse_kinematics(client, previous, pose)
        deltas = [abs(float(target) - float(source)) for source, target in zip(previous, current)]
        maximum = max(deltas)
        min_abs_j5 = min(min_abs_j5, abs(float(current[4])))
        print(f"{label}: max_joint_delta={maximum:.2f}°，J5={current[4]:.2f}°，IK={current}")
        if maximum > MAX_JOINT_DELTA_DEG:
            raise RuntimeError(f"{label}单关节最大变化{maximum:.1f}°超过{MAX_JOINT_DELTA_DEG:.1f}°")
        previous = current
        solved[label] = current
    if min_abs_j5 < 5.0:
        raise RuntimeError(f"完整路径最小|J5|={min_abs_j5:.2f}°，低于5°实验限制")
    print(f"[PASS] 完整正反链逆解通过，路径最小|J5|={min_abs_j5:.2f}°")


def main() -> int:
    args = parse_args()
    if not 1.0 <= args.arm_speed <= 10.0:
        raise ValueError("机械臂速度必须在1～10%")
    if not 1 <= args.lift_speed <= 20:
        raise ValueError("升降柱速度必须在1～20%")
    if args.arm_timeout <= 0 or args.lift_timeout <= 0:
        raise ValueError("超时时间必须大于0")

    transition_pose, place_pose, place_height, ready_pose, transition_name, final_name = load_targets(args.arm)
    side_text = "左臂" if args.arm == "left" else "右臂"
    side_upper = args.arm.upper()
    print(f"\n========== {side_text}固定点放置流程 ==========")
    print(f"配置文件: {CAPABILITY_POSE_PATH}")
    print(f"READY 6D: {ready_pose}")
    print(f"过渡预设: {transition_name}")
    print(f"TRANSITION 6D: {transition_pose}")
    print(f"最终预设: {final_name}")
    print(f"FINAL完整6D: {place_pose}")
    print(f"要求躯干高度: {place_height}mm")
    print("流程: READY -> MoveJ_P过渡位 -> MoveJ_P最终放置位 -> 松爪 -> MoveJ_P过渡位 -> MoveJ_P READY")

    sys.path.insert(0, str(ARM_API_ROOT))
    from realman_arm_api_api2 import RealmanArmClient

    arm_ip = ARM_CONFIG[args.arm]["ip"]
    client = RealmanArmClient(ip=arm_ip, model=args.arm, auto_connect=False)
    lift_client = client
    separate_lift_client = None
    try:
        print(f"\n连接{side_text}: {arm_ip}")
        client.connect()
        if args.arm == "right":
            separate_lift_client = RealmanArmClient(
                ip=ARM_CONFIG["left"]["ip"], model="left", auto_connect=False
            )
            print(f"连接升降柱所属左臂控制器（仅读取高度）: {ARM_CONFIG['left']['ip']}")
            separate_lift_client.connect()
            lift_client = separate_lift_client

        # 初始状态必须是：READY、400mm放置高度、夹爪正在夹持物体。
        require_pose(
            client,
            f"启动时{side_text} READY姿态",
            ready_pose,
            START_POSITION_TOLERANCE_M,
            START_ORIENTATION_TOLERANCE_DEG,
        )
        initial_height = check_lift(lift_client, place_height)
        check_gripper_holding(client)
        print(f"启动状态检查通过：躯干={initial_height}mm，{side_text}在READY，夹爪已夹持")
        preflight_chain(client, ready_pose, transition_pose, place_pose)
        if args.plan_only:
            print("\n[PLAN ONLY] 只读检查完成，不会发送运动或夹爪命令")
            return 0

        # 阶段1：READY -> TRANSITION。
        check_lift(lift_client, place_height)
        require_pose(client, f"{side_text} READY姿态", ready_pose, START_POSITION_TOLERANCE_M, START_ORIENTATION_TOLERANCE_DEG)
        check_gripper_holding(client)
        check_arm_motion(client, f"{side_text}前往过渡位", transition_pose)
        if not confirm(
            f"MOVE {side_upper} TRANSITION",
            f"确认MoveJ_P整段路径无障碍物，以{args.arm_speed:g}%移动到过渡位",
        ):
            return 0
        actual_transition_pose = execute_movej_p_monitored(
            client, transition_pose, args.arm_speed, args.arm_timeout, "MoveJ_P过渡"
        )
        print(f"{side_text}已到过渡位，实际末端6D: {actual_transition_pose}")

        # 阶段2：TRANSITION -> FINAL。
        require_pose(
            client,
            "前往最终放置位前的过渡姿态",
            transition_pose,
            ARM_POSITION_TOLERANCE_M,
            ARM_ORIENTATION_TOLERANCE_DEG,
        )
        check_lift(lift_client, place_height)
        check_gripper_holding(client)
        check_arm_motion(client, f"{side_text}从过渡位前往最终放置位", place_pose)
        if not confirm(
            f"MOVE {side_upper} PLACE",
            f"确认过渡位到最终放置位路径无障碍物，以{args.arm_speed:g}%执行MoveJ_P",
        ):
            return 0
        actual_place_pose = execute_movej_p_monitored(
            client, place_pose, args.arm_speed, args.arm_timeout, "MoveJ_P放置"
        )
        print(f"{side_text}已到放置位，实际末端6D: {actual_place_pose}")

        # 阶段4：松爪不是机械臂运动，但仍需检查最终位姿、高度、错误和夹持状态。
        require_pose(client, "松爪前放置姿态", place_pose, ARM_POSITION_TOLERANCE_M, ARM_ORIENTATION_TOLERANCE_DEG)
        check_lift(lift_client, place_height)
        check_gripper_holding(client)
        if not confirm(
            f"RELEASE {side_upper} OBJECT",
            "确认物体已由桌面可靠承托、打开夹爪不会导致物体跌落",
        ):
            return 0
        released = release_gripper_monitored(client)
        print(f"物体已松开，夹爪状态: {released}")

        # 阶段3：松爪后按反向链返回过渡位。
        require_pose(client, "回退前放置姿态", place_pose, ARM_POSITION_TOLERANCE_M, ARM_ORIENTATION_TOLERANCE_DEG)
        check_lift(lift_client, place_height)
        check_arm_motion(client, f"{side_text}从最终放置位返回过渡位", transition_pose)
        if not confirm(
            f"RETURN {side_upper} TRANSITION",
            f"确认最终放置位到过渡位路径无障碍物，以{args.arm_speed:g}%执行MoveJ_P",
        ):
            return 0
        actual_return_transition = execute_movej_p_monitored(
            client, transition_pose, args.arm_speed, args.arm_timeout, "MoveJ_P返回过渡位"
        )
        print(f"{side_text}已返回过渡位，实际末端6D: {actual_return_transition}")

        # 阶段4：TRANSITION -> READY。
        require_pose(
            client,
            "返回READY前的过渡姿态",
            transition_pose,
            ARM_POSITION_TOLERANCE_M,
            ARM_ORIENTATION_TOLERANCE_DEG,
        )
        check_lift(lift_client, place_height)
        check_arm_motion(client, f"{side_text}从过渡位返回READY", ready_pose)
        if not confirm(
            f"RETURN {side_upper} READY",
            f"确认过渡位到READY路径无障碍物，以{args.arm_speed:g}%返回",
        ):
            return 0
        actual_ready_pose = execute_movej_p_monitored(
            client, ready_pose, args.arm_speed, args.arm_timeout, "MoveJ_P回READY"
        )
        print(f"{side_text}已回READY，实际末端6D: {actual_ready_pose}")
        require_pose(client, f"流程结束{side_text} READY姿态", ready_pose, ARM_POSITION_TOLERANCE_M, ARM_ORIENTATION_TOLERANCE_DEG)
        check_lift(lift_client, place_height)
        final_gripper = client.get_gripper_state()
        if final_gripper.error != 0 or final_gripper.actpos < 990:
            raise RuntimeError(f"流程结束夹爪状态异常: {final_gripper}")
        print("\n[COMPLETE] 放置流程完成")
        print(f"躯干保持{place_height}mm，{side_text}已回READY，夹爪保持打开")
        return 0
    finally:
        if separate_lift_client is not None:
            separate_lift_client.disconnect()
        client.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n用户中止。", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
