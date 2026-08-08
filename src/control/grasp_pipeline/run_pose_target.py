#!/usr/bin/env python3
"""将网页6D物体位姿转换到机械臂基座系，并低速移动至安全观察点。

输入位姿约定：camera_T_object = [x_mm, y_mm, z_mm, rx, ry, rz]，
其中姿态为XYZ欧拉角（弧度）。执行前必须通过数值、工作空间、运动距离和
瑞尔曼逆运动学检查，并在终端输入完整确认词。
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parent
HAND_EYE_ROOT = Path("/home/lh/WRC/src/hand_eye_calibration/datasets")
ARM_API_ROOT = Path("/home/lh/robot_api/arm_api_new")
POSE_ROOT = ROOT.parent / "initial_pose_console" / "config" / "poses"
PRESET_PATH = ROOT / "config" / "preset_poses.yaml"

ARM_IPS = {"left": "169.254.128.18", "right": "169.254.128.19"}
DYNAMIC_RESULTS = {
    "left": HAND_EYE_ROOT / "dynamic_eye_to_hand_left/results/dynamic_160_40_five_repeats.json",
    "right": HAND_EYE_ROOT / "dynamic_eye_to_hand_right/results/dynamic_160_38_five_repeats.json",
}
EYE_IN_HAND_RESULTS = {
    "left": HAND_EYE_ROOT / "eye_in_hand_left/results/eye_in_hand_20_5_five_repeats.json",
    "right": HAND_EYE_ROOT / "eye_in_hand_right/results/eye_in_hand_20_5_five_repeats.json",
}
STANDOFF_M = 0.150
SPEED_PERCENT = 10
LINEAR_SPEED_PERCENT = 5
FINAL_APPROACH_SPEED_PERCENT = 4
TCP_OFFSET_FLANGE_M = np.array([0.0, 0.0, 0.130])
FINAL_STOP_M = 0.010
GRIPPER_SPEED = 100
MOVEL_TIMEOUT_S = 60.0
MOVEL_POLL_S = 0.2
MOVEL_POSITION_TOLERANCE_M = 0.008
MOVEL_ORIENTATION_TOLERANCE_DEG = 2.0
MOVEL_STABLE_SAMPLES = 3
MOVEL_STALL_TIMEOUT_S = 4.0
MOVEL_PROGRESS_M = 0.001
FINAL_APPROACH_TIMEOUT_S = 15.0
GRIPPER_FORCE = 200
GRIPPER_CLOSE_TIMEOUT_S = 15.0
RETURN_LIFT_M = 0.050
RETURN_LIFT_AXIS_BASE = np.array([0.0, 0.0, 1.0])
RETURN_LINEAR_SPEED_PERCENT = 12
RETURN_MOVEJ_SPEED_PERCENT = 12
RETURN_MOVEL_TIMEOUT_S = 90.0
RETURN_MOVEJ_TIMEOUT_S = 60.0
MAX_MOVE_M = 0.600
MAX_JOINT_DELTA_DEG = 90.0
BASE_RADIUS_LIMIT_M = 1.200
BASE_Z_LIMIT_M = (-0.300, 1.200)
CAMERA_DEPTH_LIMIT_M = (0.100, 2.000)
FLOAT_PATTERN = re.compile(
    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="6D位姿坐标转换、安全检查和低速预抓取测试")
    parser.add_argument("--mode", choices=("eye_to_hand", "eye_in_hand"))
    parser.add_argument("--arm", choices=("left", "right"))
    parser.add_argument(
        "--shelf-level",
        choices=("upper", "middle"),
        default="upper",
        help="货架层级；默认upper以保持原有行为",
    )
    parser.add_argument(
        "--preset",
        help="精确指定统一YAML拍照预设；提供后覆盖--shelf-level自动生成的名称",
    )
    parser.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--standoff", type=float, default=STANDOFF_M, help="距物体的停止距离，单位m")
    parser.add_argument(
        "--return-lift-mm",
        type=float,
        default=RETURN_LIFT_M * 1000.0,
        help="抓取后基座系+Z方向上升距离，范围30～50mm",
    )
    return parser.parse_args()


def resolve_photo_preset_name(
    mode: str,
    arm: str,
    shelf_level: str,
    override: str | None,
) -> str:
    if override:
        return override
    return f"shelf_{shelf_level}_photo_{mode}_{arm}"


def read_pose_interactively(stream=None) -> list[float]:
    """从终端读取6D，兼容单行或网页逐行/逗号格式粘贴。"""
    if stream is None:
        stream = sys.stdin
    print(
        "粘贴6D位姿 x_mm y_mm z_mm rx ry rz。\n"
        "支持单行空格/逗号格式，也支持每个数单独一行；读取满6个数后自动继续："
    )
    values: list[float] = []
    while len(values) < 6:
        line = stream.readline()
        if line == "":
            raise ValueError(f"输入提前结束：当前只读取到{len(values)}个数")
        if not line.strip():
            if values:
                raise ValueError(f"空行结束输入，但当前只读取到{len(values)}个数")
            continue
        numbers = [float(token) for token in FLOAT_PATTERN.findall(line)]
        if not numbers:
            # 网页复制格式可能把逗号单独放在一行，这种行直接忽略。
            continue
        values.extend(numbers)
        if len(values) > 6:
            raise ValueError(f"6D位姿只能包含6个数，实际读取到{len(values)}个")
        print(f"已读取 {len(values)}/6: {values}")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("6D位姿必须全部是有限数值")
    return values


def prompt_missing(args: argparse.Namespace) -> tuple[str, str, list[float]]:
    mode = args.mode or input("模式 eye_to_hand / eye_in_hand: ").strip()
    arm = args.arm or input("执行机械臂 left / right: ").strip()
    if mode not in {"eye_to_hand", "eye_in_hand"}:
        raise ValueError("模式必须是 eye_to_hand 或 eye_in_hand")
    if arm not in {"left", "right"}:
        raise ValueError("机械臂必须是 left 或 right")
    if mode == "eye_in_hand" and arm == "right" and not EYE_IN_HAND_RESULTS["right"].is_file():
        raise RuntimeError("右臂眼在手上标定结果不存在，禁止执行")
    if args.pose is None:
        values = read_pose_interactively()
    else:
        values = list(args.pose)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("必须输入6个有限数值")
    return mode, arm, values


def load_json(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"缺少文件: {path}")
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def euler_pose_to_transform(position_m, euler_xyz_rad) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", euler_xyz_rad, degrees=False).as_matrix()
    result[:3, 3] = np.asarray(position_m, dtype=float)
    return result


def transform_to_robot_pose(transform: np.ndarray) -> list[float]:
    euler = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz", degrees=False)
    return np.r_[transform[:3, 3], euler].astype(float).tolist()


def pose6_model_to_transform(value) -> np.ndarray:
    """动态模型内部pose6顺序是[旋转向量, 平移]。"""
    value = np.asarray(value, dtype=float)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(value[:3]).as_matrix()
    result[:3, 3] = value[3:6]
    return result


def twist_exp(value) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    omega, velocity = value[:3], value[3:]
    theta = float(np.linalg.norm(omega))
    skew = np.array([
        [0.0, -omega[2], omega[1]],
        [omega[2], 0.0, -omega[0]],
        [-omega[1], omega[0], 0.0],
    ])
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(omega).as_matrix()
    if theta < 1e-10:
        jacobian = np.eye(3) + 0.5 * skew
    else:
        jacobian = (
            np.eye(3)
            + (1.0 - np.cos(theta)) / theta**2 * skew
            + (theta - np.sin(theta)) / theta**3 * (skew @ skew)
        )
    result[:3, 3] = jacobian @ velocity
    return result


def dynamic_base_to_camera(arm: str, yaw: float, pitch: float) -> tuple[np.ndarray, Path]:
    path = DYNAMIC_RESULTS[arm]
    parameters = np.asarray(load_json(path)["best_parameters"], dtype=float)
    if parameters.shape != (24,):
        raise ValueError(f"动态模型参数长度异常: {parameters.shape}")
    dy = (float(yaw) - 500.0) / 100.0
    dp = (float(pitch) - 450.0) / 50.0
    transform = (
        pose6_model_to_transform(parameters[:6])
        @ twist_exp(parameters[6:12] * dy)
        @ twist_exp(parameters[12:18] * dp)
    )
    return transform, path


def load_eye_in_hand(arm: str) -> tuple[np.ndarray, Path]:
    path = EYE_IN_HAND_RESULTS[arm]
    transform = np.asarray(load_json(path)["best_gripper_T_camera"], dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("眼在手上矩阵不是4x4")
    return transform, path


def load_photo_pose(arm: str, preset_name: str) -> tuple[list[float], Path]:
    if not PRESET_PATH.is_file():
        raise FileNotFoundError(f"统一预设文件不存在: {PRESET_PATH}")
    with PRESET_PATH.open("r", encoding="utf-8") as handle:
        registry = yaml.safe_load(handle)
    if not isinstance(registry, dict) or not isinstance(registry.get("presets"), dict):
        raise ValueError(f"统一预设文件结构异常: {PRESET_PATH}")
    preset = registry["presets"].get(preset_name)
    if not isinstance(preset, dict):
        raise ValueError(f"统一 YAML 中不存在拍照预设: {preset_name}")
    arm_component = preset.get(f"{arm}_arm")
    if not isinstance(arm_component, dict):
        raise ValueError(f"拍照预设缺少{arm}_arm字段: {preset_name}")
    pose = arm_component.get("pose_6d")
    if not isinstance(pose, list) or len(pose) != 6:
        raise ValueError(f"拍照预设pose_6d格式异常: {preset_name}")
    values = [float(value) for value in pose]
    if not np.all(np.isfinite(values)):
        raise ValueError(f"拍照预设包含非有限数值: {preset_name}")
    applied = preset.get("apply_components")
    if isinstance(applied, list) and f"{arm}_arm" not in applied:
        raise ValueError(f"拍照预设未将{arm}_arm列入apply_components: {preset_name}")
    return values, PRESET_PATH


def make_pose_with_position(position: np.ndarray, rotation: np.ndarray) -> list[float]:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform_to_robot_pose(transform)


def make_return_poses(
    grasp_position: np.ndarray,
    rotation: np.ndarray,
    approach_axis: np.ndarray,
    standoff: float,
    lift_height: float,
    photo_pose: list[float],
) -> dict[str, object]:
    lift_delta = RETURN_LIFT_AXIS_BASE * lift_height
    lower_position = np.asarray(grasp_position, dtype=float) - approach_axis * standoff
    return {
        "lift": make_pose_with_position(np.asarray(grasp_position) + lift_delta, rotation),
        "retreat": make_pose_with_position(lower_position + lift_delta, rotation),
        "lower": make_pose_with_position(lower_position, rotation),
        "photo": list(photo_pose),
    }


def validate_position_target(name: str, position: np.ndarray, errors: list[str]) -> None:
    radius = float(np.linalg.norm(position))
    if radius > BASE_RADIUS_LIMIT_M:
        errors.append(f"{name}目标距基座原点{radius:.3f}m，超过{BASE_RADIUS_LIMIT_M:.3f}m限制")
    if not BASE_Z_LIMIT_M[0] <= float(position[2]) <= BASE_Z_LIMIT_M[1]:
        errors.append(f"{name}目标Z={position[2]:.3f}m超出{BASE_Z_LIMIT_M}")


def validate_return_targets(
    poses: dict[str, object],
    current_position: np.ndarray,
    errors: list[str],
) -> None:
    pose_positions = {
        name: np.asarray(value[:3], dtype=float)
        for name, value in poses.items()
    }
    for name, position in pose_positions.items():
        validate_position_target(f"回退-{name}", position, errors)
    for first, second in (("lift", "retreat"), ("retreat", "lower")):
        distance = float(np.linalg.norm(pose_positions[second] - pose_positions[first]))
        if distance <= 0.0 or distance > MAX_MOVE_M:
            errors.append(f"回退{first}->{second}距离{distance:.3f}m异常")
    return_distance = float(np.linalg.norm(pose_positions["photo"] - pose_positions["lower"]))
    if return_distance <= 0.0 or return_distance > MAX_MOVE_M:
        errors.append(f"回退到拍照位MoveJ_P距离{return_distance:.3f}m异常")
    initial_distance = float(np.linalg.norm(pose_positions["lift"] - current_position))
    if initial_distance > MAX_MOVE_M:
        errors.append(f"回退起始位置距离{initial_distance:.3f}m超过{MAX_MOVE_M:.3f}m限制")


def validate_transform(name: str, transform: np.ndarray, errors: list[str]) -> None:
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        errors.append(f"{name} 不是有效4x4有限矩阵")
        return
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        errors.append(f"{name} 旋转矩阵不正交")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        errors.append(f"{name} 旋转矩阵行列式不为1")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        errors.append(f"{name} 齐次矩阵末行错误")


def nonzero_robot_errors(value) -> list[str]:
    found = []
    def visit(item, path="err"):
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


def inverse_kinematics(client, current_joints, target_pose) -> list[float]:
    from Robotic_Arm.rm_ctypes_wrap import rm_inverse_kinematics_params_t

    q_in = list(float(value) for value in current_joints)
    q_in = (q_in + [0.0] * 7)[:7]
    params = rm_inverse_kinematics_params_t(q_in=q_in, q_pose=target_pose, flag=1)
    raw = client.raw_call("rm_algo_inverse_kinematics", params, check=False)
    if not isinstance(raw, tuple) or len(raw) != 2:
        raise RuntimeError(f"逆运动学返回格式异常: {raw!r}")
    code, joints = raw
    if int(code) != 0:
        raise RuntimeError(f"目标不可达，逆运动学错误码={code}")
    return [float(value) for value in joints]


def wait_gripper_open(client, timeout: float = 8.0, minimum_position: int = 990):
    """轮询原生因时夹爪，直到确认已打开。"""
    import time

    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = client.get_gripper_state()
        if last.error != 0:
            raise RuntimeError(f"夹爪错误码={last.error}")
        if last.enable_state == 1 and last.actpos >= minimum_position:
            return last
        time.sleep(0.2)
    raise RuntimeError(f"夹爪未在{timeout:.1f}s内打开，最后状态={last}")


def stop_motion_safely(client) -> None:
    try:
        client.move_stop()
    except Exception as exc:
        print(f"[WARN] 发送机械臂停止指令失败: {exc}", file=sys.stderr)


def execute_movel_monitored(
    client,
    target_pose: list[float],
    *,
    speed_percent: float = LINEAR_SPEED_PERCENT,
    timeout_s: float = MOVEL_TIMEOUT_S,
) -> list[float]:
    """非阻塞发送MoveL，并通过实际位姿、错误码、超时和停滞主动判定结果。"""
    target_position = np.asarray(target_pose[:3], dtype=float)
    target_rotation = Rotation.from_euler("xyz", target_pose[3:], degrees=False)
    deadline = time.monotonic() + timeout_s
    last_progress_time = time.monotonic()
    best_position_error = math.inf
    stable_samples = 0
    last_report_time = 0.0
    command_sent = False

    try:
        client.movel(
            target_pose,
            v=speed_percent,
            r=0,
            trajectory_connect=0,
            block=False,
        )
        command_sent = True

        while True:
            now = time.monotonic()
            state = client.get_state()
            robot_errors = nonzero_robot_errors(state.err)
            if robot_errors:
                raise RuntimeError("MoveL期间机械臂错误: " + ", ".join(robot_errors))

            actual_pose = state.pose.as_list()
            actual_position = np.asarray(actual_pose[:3], dtype=float)
            actual_rotation = Rotation.from_euler("xyz", actual_pose[3:], degrees=False)
            position_error = float(np.linalg.norm(actual_position - target_position))
            orientation_error_deg = float(
                np.degrees((target_rotation.inv() * actual_rotation).magnitude())
            )

            if position_error <= best_position_error - MOVEL_PROGRESS_M:
                best_position_error = position_error
                last_progress_time = now

            if now - last_report_time >= 0.5:
                print(
                    f"MoveL监控: 位置误差={position_error * 1000.0:.1f} mm, "
                    f"姿态误差={orientation_error_deg:.2f}°, 稳定={stable_samples}/{MOVEL_STABLE_SAMPLES}"
                )
                last_report_time = now

            if (
                position_error <= MOVEL_POSITION_TOLERANCE_M
                and orientation_error_deg <= MOVEL_ORIENTATION_TOLERANCE_DEG
            ):
                stable_samples += 1
                if stable_samples >= MOVEL_STABLE_SAMPLES:
                    return actual_pose
            else:
                stable_samples = 0

            if now >= deadline:
                raise TimeoutError(
                    f"MoveL超过{timeout_s:.0f}s仍未到位，"
                    f"位置误差={position_error * 1000.0:.1f} mm，"
                    f"姿态误差={orientation_error_deg:.2f}°"
                )
            if (
                position_error > MOVEL_POSITION_TOLERANCE_M
                and now - last_progress_time >= MOVEL_STALL_TIMEOUT_S
            ):
                raise RuntimeError(
                    f"MoveL连续{MOVEL_STALL_TIMEOUT_S:.0f}s无至少"
                    f"{MOVEL_PROGRESS_M * 1000.0:.0f} mm进展，"
                    f"当前位置误差={position_error * 1000.0:.1f} mm"
                )
            time.sleep(MOVEL_POLL_S)
    except BaseException:
        if command_sent:
            stop_motion_safely(client)
        raise


def execute_movej_p_monitored(
    client,
    target_pose: list[float],
    *,
    speed_percent: float = RETURN_MOVEJ_SPEED_PERCENT,
    timeout_s: float = RETURN_MOVEJ_TIMEOUT_S,
) -> list[float]:
    target_position = np.asarray(target_pose[:3], dtype=float)
    target_rotation = Rotation.from_euler("xyz", target_pose[3:], degrees=False)
    deadline = time.monotonic() + timeout_s
    stable_samples = 0
    last_report_time = 0.0
    command_sent = False
    try:
        client.movej_p(
            target_pose,
            v=speed_percent,
            r=0,
            trajectory_connect=0,
            block=False,
        )
        command_sent = True
        while True:
            now = time.monotonic()
            state = client.get_state()
            robot_errors = nonzero_robot_errors(state.err)
            if robot_errors:
                raise RuntimeError("回拍照位MoveJ_P期间机械臂错误: " + ", ".join(robot_errors))
            actual_pose = state.pose.as_list()
            actual_position = np.asarray(actual_pose[:3], dtype=float)
            actual_rotation = Rotation.from_euler("xyz", actual_pose[3:], degrees=False)
            position_error = float(np.linalg.norm(actual_position - target_position))
            orientation_error_deg = float(
                np.degrees((target_rotation.inv() * actual_rotation).magnitude())
            )
            if now - last_report_time >= 0.5:
                print(
                    f"回拍照位MoveJ_P监控: 位置误差={position_error * 1000.0:.1f} mm, "
                    f"姿态误差={orientation_error_deg:.2f}°"
                )
                last_report_time = now
            if (
                position_error <= MOVEL_POSITION_TOLERANCE_M
                and orientation_error_deg <= MOVEL_ORIENTATION_TOLERANCE_DEG
            ):
                stable_samples += 1
                if stable_samples >= MOVEL_STABLE_SAMPLES:
                    return actual_pose
            else:
                stable_samples = 0
            if now >= deadline:
                raise TimeoutError(
                    f"回拍照位MoveJ_P超过{timeout_s:.0f}s仍未到位，"
                    f"位置误差={position_error * 1000.0:.1f} mm，"
                    f"姿态误差={orientation_error_deg:.2f}°"
                )
            time.sleep(MOVEL_POLL_S)
    except BaseException:
        if command_sent:
            stop_motion_safely(client)
        raise


def execute_gripper_pick_monitored(client):
    """持续力控闭合夹爪，并以mode=6和稳定开度确认接触夹持。"""
    initial_state = client.get_gripper_state()
    initial_position = initial_state.actpos
    command_sent = False
    try:
        client.gripper_pick_keep(
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            block=False,
            timeout=1,
        )
        command_sent = True
        deadline = time.monotonic() + GRIPPER_CLOSE_TIMEOUT_S
        previous_position = None
        stable_samples = 0
        movement_observed = False
        last_report_time = 0.0
        last_state = None

        while time.monotonic() < deadline:
            last_state = client.get_gripper_state()
            if last_state.enable_state != 1:
                raise RuntimeError(f"夹爪闭合期间失去使能: {last_state}")
            if last_state.status != 1:
                raise RuntimeError(f"夹爪闭合期间离线: {last_state}")
            if last_state.error != 0:
                raise RuntimeError(f"夹爪闭合错误码={last_state.error}")

            if abs(last_state.actpos - initial_position) >= 10:
                movement_observed = True
            if (
                movement_observed
                and previous_position is not None
                and abs(last_state.actpos - previous_position) <= 3
            ):
                stable_samples += 1
            else:
                stable_samples = 0
            previous_position = last_state.actpos

            now = time.monotonic()
            if now - last_report_time >= 0.5:
                print(
                    f"夹爪监控: actpos={last_state.actpos}, force={last_state.current_force}, "
                    f"mode={last_state.mode}, 已运动={movement_observed}, 稳定={stable_samples}/5"
                )
                last_report_time = now

            if stable_samples >= 5:
                if last_state.actpos <= 50:
                    raise RuntimeError("夹爪接近完全闭合，未确认夹到物体")
                if last_state.actpos >= 950:
                    raise RuntimeError("夹爪仍接近全开位置，未完成夹取")
                if last_state.mode != 6:
                    raise RuntimeError(
                        f"夹爪已停止但mode={last_state.mode}，"
                        "未确认为力控接触停止(mode=6)"
                    )
                return last_state
            time.sleep(0.2)

        raise TimeoutError(
            f"夹爪闭合超过{GRIPPER_CLOSE_TIMEOUT_S:.0f}s未稳定，最后状态={last_state}"
        )
    except BaseException:
        if command_sent:
            try:
                client.gripper_release(speed=GRIPPER_SPEED, block=False, timeout=1)
                print("[WARN] 夹爪闭合未确认成功，已发送重新打开命令", file=sys.stderr)
            except Exception as exc:
                print(f"[WARN] 夹爪重新打开命令失败: {exc}", file=sys.stderr)
        raise


def print_matrix(name: str, value: np.ndarray) -> None:
    print(f"\n{name}:")
    for row in value:
        print("  [" + ", ".join(f"{number: .9f}" for number in row) + "]")


def main() -> int:
    args = parse_args()
    mode, arm, pose = prompt_missing(args)
    standoff = float(args.standoff)
    if not 0.10 <= standoff <= 0.30:
        raise ValueError("首次测试停止距离必须在0.10～0.30m")
    lift_height = float(args.return_lift_mm) / 1000.0
    if not 0.030 <= lift_height <= 0.050:
        raise ValueError("抓取后上升距离必须在30～50mm范围内")
    photo_preset_name = resolve_photo_preset_name(
        mode,
        arm,
        args.shelf_level,
        args.preset,
    )
    photo_pose, photo_pose_path = load_photo_pose(arm, photo_preset_name)

    camera_xyz_m = np.asarray(pose[:3], dtype=float) / 1000.0
    camera_euler = np.asarray(pose[3:], dtype=float)
    camera_to_object = euler_pose_to_transform(camera_xyz_m, camera_euler)

    errors: list[str] = []
    warnings: list[str] = []
    if not CAMERA_DEPTH_LIMIT_M[0] <= camera_xyz_m[2] <= CAMERA_DEPTH_LIMIT_M[1]:
        errors.append(f"相机Z深度{camera_xyz_m[2]:.3f}m超出{CAMERA_DEPTH_LIMIT_M}")
    validate_transform("camera_T_object", camera_to_object, errors)

    sys.path.insert(0, str(ARM_API_ROOT))
    from realman_arm_api_api2 import RealmanArmClient

    client = RealmanArmClient(ip=ARM_IPS[arm], model=arm, auto_connect=False)
    try:
        print(f"\n连接{arm}臂: {ARM_IPS[arm]}")
        client.connect()
        state = client.get_state()
        base_to_gripper = euler_pose_to_transform(state.pose.as_list()[:3], state.pose.as_list()[3:])
        validate_transform("base_T_gripper_current", base_to_gripper, errors)
        robot_errors = nonzero_robot_errors(state.err)
        if robot_errors:
            errors.append("机械臂错误: " + ", ".join(robot_errors))

        if mode == "eye_to_hand":
            head_pose_path = POSE_ROOT / "head_initial_photo_pose.json"
            head_pose = load_json(head_pose_path)
            yaw = float(head_pose["head"]["yaw"])
            pitch = float(head_pose["head"]["pitch"])
            base_to_camera, calibration_path = dynamic_base_to_camera(arm, yaw, pitch)
            base_to_object = base_to_camera @ camera_to_object
            intermediate = [("base_T_camera", base_to_camera)]
            warnings.append(f"动态模型使用保存的头部读数 yaw={yaw:g}, pitch={pitch:g}；脚本未读取头部实际值")
        else:
            gripper_to_camera, calibration_path = load_eye_in_hand(arm)
            base_to_object = base_to_gripper @ gripper_to_camera @ camera_to_object
            intermediate = [("base_T_gripper_current", base_to_gripper), ("gripper_T_camera", gripper_to_camera)]

        validate_transform("base_T_object", base_to_object, errors)
        object_position = base_to_object[:3, 3]
        current_position = base_to_gripper[:3, 3]

        # 夹爪中心位于法兰局部+Z方向130 mm；先用当前末端旋转将TCP偏移
        # 和接近轴转换到机械臂基座系。法兰抓取点 = 物体点 - base系TCP偏移。
        approach_axis = base_to_gripper[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tcp_offset_base = base_to_gripper[:3, :3] @ TCP_OFFSET_FLANGE_M
        flange_grasp_position = object_position - tcp_offset_base
        pregrasp_position = flange_grasp_position - approach_axis * standoff
        near_position = flange_grasp_position - approach_axis * FINAL_STOP_M

        pregrasp_transform = np.eye(4)
        pregrasp_transform[:3, :3] = base_to_gripper[:3, :3]
        pregrasp_transform[:3, 3] = pregrasp_position
        pregrasp_pose = transform_to_robot_pose(pregrasp_transform)

        near_transform = np.eye(4)
        near_transform[:3, :3] = base_to_gripper[:3, :3]
        near_transform[:3, 3] = near_position
        near_pose = transform_to_robot_pose(near_transform)

        grasp_transform = np.eye(4)
        grasp_transform[:3, :3] = base_to_gripper[:3, :3]
        grasp_transform[:3, 3] = flange_grasp_position
        grasp_pose = transform_to_robot_pose(grasp_transform)

        return_poses = make_return_poses(
            flange_grasp_position,
            base_to_gripper[:3, :3],
            approach_axis,
            standoff,
            lift_height,
            photo_pose,
        )
        photo_transform = euler_pose_to_transform(photo_pose[:3], photo_pose[3:])
        validate_transform("photo_pose", photo_transform, errors)

        movej_distance = float(np.linalg.norm(pregrasp_position - current_position))
        movel_distance = float(np.linalg.norm(near_position - pregrasp_position))

        for name, position in (
            ("预抓取", pregrasp_position),
            ("距抓取点10mm", near_position),
            ("完整抓取", flange_grasp_position),
        ):
            radius = float(np.linalg.norm(position))
            if radius > BASE_RADIUS_LIMIT_M:
                errors.append(f"{name}目标距基座原点{radius:.3f}m，超过{BASE_RADIUS_LIMIT_M:.3f}m限制")
            if not BASE_Z_LIMIT_M[0] <= position[2] <= BASE_Z_LIMIT_M[1]:
                errors.append(f"{name}目标Z={position[2]:.3f}m超出{BASE_Z_LIMIT_M}")
        if movej_distance > MAX_MOVE_M:
            errors.append(f"MoveJ_P移动{movej_distance:.3f}m超过{MAX_MOVE_M:.3f}m限制")
        if movel_distance <= 0 or movel_distance > standoff:
            errors.append(f"MoveL接近距离异常: {movel_distance:.3f}m")
        validate_return_targets(return_poses, flange_grasp_position, errors)
        if arm != "left":
            errors.append("当前只验证了左臂因时原生夹爪，首次夹取流程禁止用于右臂")

        gripper_state = None
        try:
            gripper_state = client.get_gripper_state()
            if gripper_state.enable_state != 1:
                errors.append(f"左臂夹爪未使能: enable_state={gripper_state.enable_state}")
            if gripper_state.error != 0:
                errors.append(f"左臂夹爪错误码={gripper_state.error}")
        except Exception as exc:
            errors.append(f"读取左臂夹爪状态失败: {exc}")

        pregrasp_ik = None
        near_ik = None
        grasp_ik = None
        return_iks = None
        joint_deltas = None
        return_joint_deltas = None
        if not errors:
            try:
                pregrasp_ik = inverse_kinematics(client, state.joints, pregrasp_pose)
                joint_deltas = [
                    abs(float(target) - float(current))
                    for current, target in zip(state.joints, pregrasp_ik)
                ]
                if joint_deltas and max(joint_deltas) > MAX_JOINT_DELTA_DEG:
                    errors.append(
                        f"逆解需要单关节最大变化{max(joint_deltas):.1f}°，"
                        f"超过首次测试限制{MAX_JOINT_DELTA_DEG:.1f}°"
                    )
                if not errors:
                    near_ik = inverse_kinematics(client, pregrasp_ik, near_pose)
                    grasp_ik = inverse_kinematics(client, near_ik, grasp_pose)
                    return_iks = {}
                    return_joint_deltas = {}
                    previous_ik = grasp_ik
                    for return_name in ("lift", "retreat", "lower", "photo"):
                        return_ik = inverse_kinematics(
                            client,
                            previous_ik,
                            return_poses[return_name],
                        )
                        deltas = [
                            abs(float(target) - float(current))
                            for current, target in zip(previous_ik, return_ik)
                        ]
                        if deltas and max(deltas) > MAX_JOINT_DELTA_DEG:
                            errors.append(
                                f"回退{return_name}逆解单关节最大变化{max(deltas):.1f}°，"
                                f"超过{MAX_JOINT_DELTA_DEG:.1f}°限制"
                            )
                        return_iks[return_name] = return_ik
                        return_joint_deltas[return_name] = deltas
                        previous_ik = return_ik
            except Exception as exc:
                errors.append(str(exc))

        print("\n========== 坐标转换结果 ==========")
        print(f"模式: {mode}, 执行机械臂: {arm}")
        print(f"标定文件: {calibration_path}")
        print_matrix("camera_T_object", camera_to_object)
        for name, matrix in intermediate:
            print_matrix(name, matrix)
        print_matrix("base_T_object", base_to_object)
        print_matrix("base_T_flange_pregrasp", pregrasp_transform)
        print_matrix("base_T_flange_stop_10mm", near_transform)
        print_matrix("base_T_flange_grasp", grasp_transform)
        print(f"\n物体基座系位置(m): {object_position.tolist()}")
        print(f"当前末端位置(m):   {current_position.tolist()}")
        print(f"法兰局部TCP偏移(m): {TCP_OFFSET_FLANGE_M.tolist()}")
        print(f"本次TCP偏移(m, base): {tcp_offset_base.tolist()}")
        print(f"本次接近方向(+Z, base): {approach_axis.tolist()}")
        print(f"理论法兰抓取位置(m): {flange_grasp_position.tolist()}")
        print(f"预抓取停止距离(m): {standoff:.3f}")
        print(f"最终保留距离(m):   {FINAL_STOP_M:.3f}")
        print(f"预抓取末端6D:      {pregrasp_pose}")
        print(f"10mm停止点末端6D:  {near_pose}")
        print(f"完整抓取点末端6D: {grasp_pose}")
        print(f"MoveJ_P距离(m):    {movej_distance:.3f}")
        print(f"MoveL距离(m):      {movel_distance:.3f}")
        print(f"当前关节角(deg):   {state.joints}")
        print(f"预抓取逆解(deg):   {pregrasp_ik}")
        print(f"10mm点逆解(deg):   {near_ik}")
        print(f"抓取点逆解(deg):   {grasp_ik}")
        print(f"关节变化绝对值:    {joint_deltas}")
        print(f"夹爪状态:          {gripper_state}")
        print(f"拍照位姿文件:      {photo_pose_path}")
        print(f"拍照预设:          {photo_preset_name}")
        print(f"拍照位姿末端6D:    {photo_pose}")
        print(f"回退上升方向(base): {RETURN_LIFT_AXIS_BASE.tolist()}")
        print(f"回退上升距离(m):    {lift_height:.3f}")
        for return_name in ("lift", "retreat", "lower", "photo"):
            print(f"回退-{return_name}末端6D: {return_poses[return_name]}")
        print(f"回退逆解(deg):      {return_iks}")
        print(f"回退关节变化绝对值:  {return_joint_deltas}")

        for warning in warnings:
            print(f"[WARN] {warning}")
        if errors:
            print("\n[REJECTED] 安全检查未通过:")
            for error in errors:
                print(f"  - {error}")
            print("不会发送任何运动命令。")
            return 2

        print("\n[PASS] 基础安全检查和逆运动学检查通过。")
        print("注意：程序没有环境模型，无法自动检查机械臂与障碍物碰撞。")
        first_confirmation = f"MOVE {arm.upper()} PREGRASP"
        confirmation = input(
            f"确认空间无人、无障碍后，输入 {first_confirmation} 打开夹爪并执行{SPEED_PERCENT}% MoveJ_P: "
        ).strip()
        if confirmation != first_confirmation:
            print("已取消，没有发送运动命令。")
            return 0

        client.configure_gripper_range(0, 1000)
        # 每次抓取都主动恢复最大开度，不能依赖上一次流程遗留的夹爪状态。
        client.gripper_release(speed=GRIPPER_SPEED, block=False, timeout=1)
        current_gripper = wait_gripper_open(client, minimum_position=990)
        print(f"夹爪已确认最大打开: actpos={current_gripper.actpos}, error={current_gripper.error}")

        client.movej_p(pregrasp_pose, v=SPEED_PERCENT, r=0, trajectory_connect=0, block=True)
        actual_pregrasp = client.get_state().pose.as_list()
        print("MoveJ_P预抓取点完成，实际末端6D:")
        print(actual_pregrasp)
        pregrasp_position_error = float(
            np.linalg.norm(np.asarray(actual_pregrasp[:3], dtype=float) - pregrasp_position)
        )
        print(f"预抓取位置误差: {pregrasp_position_error * 1000.0:.2f} mm")
        if pregrasp_position_error > 0.015:
            raise RuntimeError(
                f"预抓取实际位置误差{pregrasp_position_error * 1000.0:.1f} mm超过15 mm，拒绝MoveL"
            )

        second_confirmation = f"APPROACH {arm.upper()} 10MM"
        confirmation = input(
            f"现场确认夹爪方向、物体和直线路径后，输入 {second_confirmation} 执行{LINEAR_SPEED_PERCENT}% MoveL: "
        ).strip()
        if confirmation != second_confirmation:
            print("已停在预抓取点；没有发送MoveL，也没有闭合夹爪。")
            return 0

        actual_near = execute_movel_monitored(client, near_pose)
        print("MoveL完成，已停在理论抓取点前10 mm，实际末端6D:")
        print(actual_near)
        near_position_error = float(np.linalg.norm(np.asarray(actual_near[:3], dtype=float) - near_position))
        print(f"10mm停止点位置误差: {near_position_error * 1000.0:.2f} mm")
        third_confirmation = f"FINAL GRASP {arm.upper()}"
        confirmation = input(
            f"确认两指位于同一物体两侧且最后10mm路径安全后，输入 {third_confirmation} "
            f"执行{FINAL_APPROACH_SPEED_PERCENT}%最终接近并以"
            f"speed={GRIPPER_SPEED}, force={GRIPPER_FORCE}持续力控闭合: "
        ).strip()
        if confirmation != third_confirmation:
            print("已停在抓取点前10 mm；没有继续接近，也没有闭合夹爪。")
            return 0

        actual_grasp = execute_movel_monitored(
            client,
            grasp_pose,
            speed_percent=FINAL_APPROACH_SPEED_PERCENT,
            timeout_s=FINAL_APPROACH_TIMEOUT_S,
        )
        print("最终10 mm MoveL完成，实际末端6D:")
        print(actual_grasp)
        gripper_result = execute_gripper_pick_monitored(client)
        print("夹取完成并通过初步状态检查:")
        print(gripper_result)

        actual_return_state = client.get_state()
        actual_grasp_position = np.asarray(actual_grasp[:3], dtype=float)
        actual_grasp_rotation = Rotation.from_euler("xyz", actual_grasp[3:], degrees=False).as_matrix()
        actual_approach_axis = actual_grasp_rotation @ np.array([0.0, 0.0, 1.0])
        actual_return_poses = make_return_poses(
            actual_grasp_position,
            actual_grasp_rotation,
            actual_approach_axis,
            standoff,
            lift_height,
            photo_pose,
        )
        return_errors: list[str] = []
        actual_robot_errors = nonzero_robot_errors(actual_return_state.err)
        if actual_robot_errors:
            return_errors.append("准备回退时机械臂错误: " + ", ".join(actual_robot_errors))
        validate_return_targets(actual_return_poses, actual_grasp_position, return_errors)

        actual_return_iks = None
        actual_return_joint_deltas = None
        if not return_errors:
            try:
                actual_return_iks = {}
                actual_return_joint_deltas = {}
                previous_ik = [float(value) for value in actual_return_state.joints]
                for return_name in ("lift", "retreat", "lower", "photo"):
                    return_ik = inverse_kinematics(
                        client,
                        previous_ik,
                        actual_return_poses[return_name],
                    )
                    deltas = [
                        abs(float(target) - float(current))
                        for current, target in zip(previous_ik, return_ik)
                    ]
                    if deltas and max(deltas) > MAX_JOINT_DELTA_DEG:
                        return_errors.append(
                            f"实际回退{return_name}逆解单关节最大变化{max(deltas):.1f}°，"
                            f"超过{MAX_JOINT_DELTA_DEG:.1f}°限制"
                        )
                    actual_return_iks[return_name] = return_ik
                    actual_return_joint_deltas[return_name] = deltas
                    previous_ik = return_ik
            except Exception as exc:
                return_errors.append(str(exc))

        print("\n========== 抓取后回退预检 ==========")
        print(f"实际接近方向(base): {actual_approach_axis.tolist()}")
        print(f"竖直上升方向(base): {RETURN_LIFT_AXIS_BASE.tolist()}")
        print(f"竖直上升距离(m):    {lift_height:.3f}")
        for return_name in ("lift", "retreat", "lower", "photo"):
            print(f"实际回退-{return_name}末端6D: {actual_return_poses[return_name]}")
        print(f"实际回退逆解(deg):   {actual_return_iks}")
        print(f"实际回退关节变化:    {actual_return_joint_deltas}")
        if return_errors:
            print("\n[RETURN REJECTED] 回退安全检查未通过，机械臂保持抓取位置:")
            for error in return_errors:
                print(f"  - {error}")
            return 3

        return_confirmation = f"RETURN {arm.upper()} PHOTO"
        confirmation = input(
            f"确认先沿基座+Z上升{lift_height * 1000.0:.0f}mm、反向退出、下降同高并回拍照位，"
            f"输入 {return_confirmation}: "
        ).strip()
        if confirmation != return_confirmation:
            print("已取消回退；机械臂保持抓取位置。")
            return 0

        actual_lift = execute_movel_monitored(
            client,
            actual_return_poses["lift"],
            speed_percent=RETURN_LINEAR_SPEED_PERCENT,
            timeout_s=RETURN_MOVEL_TIMEOUT_S,
        )
        print("回退第1步完成：基座+Z上升，实际末端6D:")
        print(actual_lift)

        actual_retreat = execute_movel_monitored(
            client,
            actual_return_poses["retreat"],
            speed_percent=RETURN_LINEAR_SPEED_PERCENT,
            timeout_s=RETURN_MOVEL_TIMEOUT_S,
        )
        print("回退第2步完成：沿接近轴反向退回预抓取区域，实际末端6D:")
        print(actual_retreat)

        actual_lower = execute_movel_monitored(
            client,
            actual_return_poses["lower"],
            speed_percent=RETURN_LINEAR_SPEED_PERCENT,
            timeout_s=RETURN_MOVEL_TIMEOUT_S,
        )
        print("回退第3步完成：基座-Z下降相同高度，实际末端6D:")
        print(actual_lower)

        actual_photo = execute_movej_p_monitored(
            client,
            actual_return_poses["photo"],
            speed_percent=RETURN_MOVEJ_SPEED_PERCENT,
            timeout_s=RETURN_MOVEJ_TIMEOUT_S,
        )
        print("回退第4步完成：低速MoveJ_P回到拍照位，实际末端6D:")
        print(actual_photo)
        final_gripper = client.get_gripper_state()
        if final_gripper.error != 0:
            raise RuntimeError(f"回到拍照位后夹爪错误码={final_gripper.error}")
        print(f"回到拍照位后夹爪状态: {final_gripper}")
        print("抓取、回退和回到拍照位流程完成；当前不会抬升、放置或松开物体。")
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
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
