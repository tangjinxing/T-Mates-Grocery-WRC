#!/usr/bin/env python3
"""圆柱物体轴约束6DOF抓取实验版。

camera_T_object 先经手眼标定变换为 base_T_object。抓取姿态完全在机械臂
基座系中生成：物体绿色+Y对齐夹爪机械竖直轴local -X；拍照姿态下夹爪
local +Z在绿色轴垂直平面内的投影定义0度前向，再绕绿色轴生成双向候选。
视觉红蓝轴仅用于诊断，不参与控制。每个候选必须通过预抓取、接近、抓取、
上升、退出、下降和回拍照位的完整逆解与安全检查后才允许执行。
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import run_pose_target as legacy


OBJECT_GREEN_AXIS = 1  # OpenCV/常见可视化：红=X、绿=Y、蓝=Z
LOCAL_APPROACH_AXIS = np.array([0.0, 0.0, 1.0])
BASE_UP_AXIS = np.array([0.0, 0.0, 1.0])
MIN_J5_ABS_DEG = 5.0
ORIENTATION_CONFIG_PATH = legacy.ROOT / "config" / "grasp_orientation.yaml"


@dataclass
class Candidate:
    name: str
    rotation: np.ndarray
    approach: np.ndarray
    pregrasp_pose: list[float]
    near_pose: list[float]
    grasp_pose: list[float]
    return_poses: dict[str, object]
    iks: dict[str, list[float]]
    max_joint_delta: float
    total_joint_motion: float
    min_abs_j5: float
    reference_angle_deg: float
    score: float
    errors: list[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="圆柱物体完整6DOF候选姿态抓取测试")
    parser.add_argument("--mode", choices=("eye_to_hand", "eye_in_hand"))
    parser.add_argument("--arm", choices=("left", "right"))
    parser.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--shelf-level", choices=("upper", "middle"), default="upper")
    parser.add_argument("--preset", help="精确指定拍照预设，覆盖--shelf-level")
    parser.add_argument("--standoff", type=float, default=legacy.STANDOFF_M)
    parser.add_argument("--return-lift-mm", type=float, default=legacy.RETURN_LIFT_M * 1000.0)
    parser.add_argument(
        "--up-axis-policy",
        choices=("base-z", "measured"),
        default="measured",
        help="measured使用视觉绿色+Y轴（当前方案）；base-z仅保留用于离线对照",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="只进行坐标转换、候选评分和逆解，绝不发送运动或夹爪命令",
    )
    return parser.parse_args()


def normalize(value: np.ndarray, name: str) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError(f"{name}无法归一化")
    return value / norm


def load_orientation_profile(shelf_level: str) -> tuple[str, dict]:
    if not ORIENTATION_CONFIG_PATH.is_file():
        raise FileNotFoundError(f"缺少抓取方向配置: {ORIENTATION_CONFIG_PATH}")
    data = yaml.safe_load(ORIENTATION_CONFIG_PATH.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("抓取方向配置顶层结构无效")
    profile_name = data.get("level_profiles", {}).get(shelf_level)
    profile = data.get("profiles", {}).get(profile_name)
    if not isinstance(profile_name, str) or not isinstance(profile, dict):
        raise ValueError(f"货架层级{shelf_level}没有有效的抓取方向配置")
    minimum = float(profile.get("offset_min_deg", -20.0))
    maximum = float(profile.get("offset_max_deg", 20.0))
    step = float(profile.get("candidate_step_deg", 10.0))
    if minimum > 0.0 or maximum < 0.0 or minimum > maximum:
        raise ValueError("候选偏角范围必须包含0°")
    if not 0.0 < step <= 30.0:
        raise ValueError("候选角步长必须在0～30°")
    offsets = []
    value = minimum
    while value <= maximum + 1e-9:
        offsets.append(round(value, 6))
        value += step
    if not any(abs(value) < 1e-9 for value in offsets):
        offsets.append(0.0)
        offsets.sort()
    result = dict(profile)
    result["candidate_offsets_deg"] = offsets
    return profile_name, result


def load_arm_pose_from_preset(name: str, arm: str) -> list[float]:
    if not legacy.PRESET_PATH.is_file():
        raise FileNotFoundError(f"统一预设文件不存在: {legacy.PRESET_PATH}")
    registry = yaml.safe_load(legacy.PRESET_PATH.read_text(encoding="utf-8"))
    preset = registry.get("presets", {}).get(name) if isinstance(registry, dict) else None
    if not isinstance(preset, dict):
        raise ValueError(f"统一YAML中不存在参考预设: {name}")
    component = preset.get(f"{arm}_arm")
    pose = component.get("pose_6d") if isinstance(component, dict) else None
    if not isinstance(pose, list) or len(pose) != 6:
        raise ValueError(f"参考预设{name}缺少{arm}_arm.pose_6d")
    values = [float(value) for value in pose]
    if not np.all(np.isfinite(values)):
        raise ValueError(f"参考预设{name}包含非有限数值")
    return values


def identify_reference_axes(reference_rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray, str]:
    """从示教姿态识别哪个夹爪局部有符号轴实际朝上；接近轴固定为局部+Z。"""
    best = None
    axis_names = ("X", "Y", "Z")
    for index in range(3):
        for sign in (1.0, -1.0):
            local = np.eye(3)[:, index] * sign
            alignment = float((reference_rotation @ local) @ BASE_UP_AXIS)
            item = (alignment, local, f"{'+' if sign > 0 else '-'}{axis_names[index]}")
            if best is None or item[0] > best[0]:
                best = item
    assert best is not None
    local_up = best[1]
    if abs(float(local_up @ LOCAL_APPROACH_AXIS)) > 1e-6:
        raise ValueError(
            f"示教姿态识别的向上轴{best[2]}与局部+Z接近轴重合，"
            "无法同时定义竖直和接近方向"
        )
    reference_approach = normalize(reference_rotation @ LOCAL_APPROACH_AXIS, "示教接近方向")
    return local_up, reference_approach, best[2]


def rotation_from_axis_mapping(
    local_up: np.ndarray,
    world_up: np.ndarray,
    world_approach_hint: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """构造R，使夹爪有符号向上轴对齐world_up，局部+Z对齐水平接近方向。"""
    world_up = normalize(world_up, "物体绿色向上轴")
    projected = world_approach_hint - world_up * float(world_approach_hint @ world_up)
    world_approach = normalize(projected, "候选水平接近轴")
    local_side = normalize(np.cross(local_up, LOCAL_APPROACH_AXIS), "夹爪局部第三轴")
    world_side = normalize(np.cross(world_up, world_approach), "目标第三轴")
    local_basis = np.column_stack((local_up, LOCAL_APPROACH_AXIS, local_side))
    world_basis = np.column_stack((world_up, world_approach, world_side))
    rotation = world_basis @ local_basis.T
    if np.linalg.det(rotation) < 0.0:
        raise ValueError("轴映射产生了反射矩阵")
    return rotation, world_approach


def rotation_distance_deg(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.degrees((Rotation.from_matrix(first).inv() * Rotation.from_matrix(second)).magnitude()))


def build_candidate(
    client,
    name: str,
    rotation: np.ndarray,
    approach: np.ndarray,
    object_position: np.ndarray,
    current_joints: list[float],
    reference_rotation: np.ndarray,
    photo_pose: list[float],
    standoff: float,
    lift_height: float,
) -> Candidate:
    errors: list[str] = []
    tcp_offset = rotation @ legacy.TCP_OFFSET_FLANGE_M
    grasp_position = object_position - tcp_offset
    pregrasp_position = grasp_position - approach * standoff
    near_position = grasp_position - approach * legacy.FINAL_STOP_M
    pregrasp_pose = legacy.make_pose_with_position(pregrasp_position, rotation)
    near_pose = legacy.make_pose_with_position(near_position, rotation)
    grasp_pose = legacy.make_pose_with_position(grasp_position, rotation)
    return_poses = legacy.make_return_poses(
        grasp_position, rotation, approach, standoff, lift_height, photo_pose
    )

    for label, position in (
        ("预抓取", pregrasp_position),
        ("10mm停止点", near_position),
        ("抓取点", grasp_position),
    ):
        legacy.validate_position_target(f"{name}-{label}", position, errors)
    legacy.validate_return_targets(return_poses, grasp_position, errors)

    iks: dict[str, list[float]] = {}
    max_delta = math.inf
    total_joint_motion = math.inf
    min_abs_j5 = 0.0
    if not errors:
        stage = "pregrasp"
        try:
            sequence = [
                ("pregrasp", pregrasp_pose),
                ("near", near_pose),
                ("grasp", grasp_pose),
                ("lift", return_poses["lift"]),
                ("retreat", return_poses["retreat"]),
                ("lower", return_poses["lower"]),
                ("photo", return_poses["photo"]),
            ]
            previous = [float(value) for value in current_joints]
            deltas = []
            total_joint_motion = 0.0
            j5_values = []
            for stage, pose in sequence:
                solved = legacy.inverse_kinematics(client, previous, pose)
                stage_deltas = [abs(float(b) - float(a)) for a, b in zip(previous, solved)]
                stage_max = max(stage_deltas) if stage_deltas else 0.0
                deltas.append(stage_max)
                total_joint_motion += float(sum(stage_deltas))
                if stage_max > legacy.MAX_JOINT_DELTA_DEG:
                    errors.append(
                        f"{stage}单关节最大变化{stage_max:.1f}°超过"
                        f"{legacy.MAX_JOINT_DELTA_DEG:.1f}°"
                    )
                if len(solved) >= 5:
                    j5_values.append(abs(float(solved[4])))
                iks[stage] = solved
                previous = solved
            max_delta = max(deltas) if deltas else 0.0
            min_abs_j5 = min(j5_values) if j5_values else 0.0
            if min_abs_j5 < MIN_J5_ABS_DEG:
                errors.append(
                    f"路径最小|J5|={min_abs_j5:.1f}°，低于实验限制{MIN_J5_ABS_DEG:.1f}°"
                )
        except Exception as exc:
            errors.append(f"{stage}阶段逆解失败: {exc}")

    reference_angle = rotation_distance_deg(reference_rotation, rotation)
    # 参考朝向优先，其次关节变化；靠近奇异点时给予连续惩罚。
    singular_penalty = 1000.0 if min_abs_j5 <= 0.0 else 50.0 / min_abs_j5
    # 安全检查是硬门槛；通过后优先0度附近、较小最大关节变化和较短全过程运动。
    score = (
        reference_angle
        + max_delta * 0.5
        + total_joint_motion * 0.1
        + singular_penalty
    )
    if errors:
        score = math.inf
    return Candidate(
        name=name,
        rotation=rotation,
        approach=approach,
        pregrasp_pose=pregrasp_pose,
        near_pose=near_pose,
        grasp_pose=grasp_pose,
        return_poses=return_poses,
        iks=iks,
        max_joint_delta=max_delta,
        total_joint_motion=total_joint_motion,
        min_abs_j5=min_abs_j5,
        reference_angle_deg=reference_angle,
        score=score,
        errors=errors,
    )


def main() -> int:
    args = parse_args()
    mode, arm, pose = legacy.prompt_missing(args)
    standoff = float(args.standoff)
    if not 0.10 <= standoff <= 0.30:
        raise ValueError("停止距离必须在0.10～0.30m")
    lift_height = float(args.return_lift_mm) / 1000.0
    if not 0.030 <= lift_height <= 0.050:
        raise ValueError("抓取后上升距离必须在30～50mm")

    photo_preset = legacy.resolve_photo_preset_name(mode, arm, args.shelf_level, args.preset)
    photo_pose, photo_path = legacy.load_photo_pose(arm, photo_preset)
    orientation_profile_name, orientation_profile = load_orientation_profile(args.shelf_level)
    camera_to_object = legacy.euler_pose_to_transform(
        np.asarray(pose[:3], dtype=float) / 1000.0,
        np.asarray(pose[3:], dtype=float),
    )
    base_errors: list[str] = []
    legacy.validate_transform("camera_T_object", camera_to_object, base_errors)
    if not legacy.CAMERA_DEPTH_LIMIT_M[0] <= float(pose[2]) / 1000.0 <= legacy.CAMERA_DEPTH_LIMIT_M[1]:
        base_errors.append("相机深度超出允许范围")

    sys.path.insert(0, str(legacy.ARM_API_ROOT))
    from realman_arm_api_api2 import RealmanArmClient

    client = RealmanArmClient(ip=legacy.ARM_IPS[arm], model=arm, auto_connect=False)
    try:
        print(f"连接{arm}臂: {legacy.ARM_IPS[arm]}")
        client.connect()
        state = client.get_state()
        robot_errors = legacy.nonzero_robot_errors(state.err)
        if robot_errors:
            base_errors.append("机械臂错误: " + ", ".join(robot_errors))
        base_to_gripper = legacy.euler_pose_to_transform(
            state.pose.as_list()[:3], state.pose.as_list()[3:]
        )
        if mode == "eye_to_hand":
            head = legacy.load_json(legacy.POSE_ROOT / "head_initial_photo_pose.json")
            base_to_camera, calibration_path = legacy.dynamic_base_to_camera(
                arm, float(head["head"]["yaw"]), float(head["head"]["pitch"])
            )
            base_to_object = base_to_camera @ camera_to_object
        else:
            gripper_to_camera, calibration_path = legacy.load_eye_in_hand(arm)
            # 本实验允许先拍照、再拖动到示教参考姿态并只读该姿态。因此当前末端
            # 已不一定是拍照瞬间末端；按本次三组数据约定，使用当前层YAML拍照
            # 位姿构造base_T_gripper_at_capture。当前实际关节仅作为逆解起点。
            base_to_gripper_at_capture = legacy.euler_pose_to_transform(
                photo_pose[:3], photo_pose[3:]
            )
            base_to_object = (
                base_to_gripper_at_capture @ gripper_to_camera @ camera_to_object
            )
        legacy.validate_transform("base_T_object", base_to_object, base_errors)

        object_rotation = base_to_object[:3, :3]
        object_position = base_to_object[:3, 3]
        red = normalize(object_rotation[:, 0], "物体红色+X轴")
        green_measured = normalize(object_rotation[:, OBJECT_GREEN_AXIS], "物体绿色+Y轴")
        blue = normalize(object_rotation[:, 2], "物体蓝色+Z轴")
        if float(green_measured @ BASE_UP_AXIS) <= 0.0:
            base_errors.append(
                "物体绿色+Y轴没有朝向机械臂基座+Z；禁止自动翻转"
            )
        green_tilt_deg = float(np.degrees(np.arccos(
            np.clip(green_measured @ BASE_UP_AXIS, -1.0, 1.0)
        )))
        max_green_tilt_deg = float(orientation_profile.get("max_green_tilt_deg", 20.0))
        if args.up_axis_policy == "measured":
            if green_tilt_deg > max_green_tilt_deg:
                base_errors.append(
                    f"measured策略下绿色轴倾斜{green_tilt_deg:.1f}°超过"
                    f"{max_green_tilt_deg:.1f}°限制"
                )
            green_control = green_measured.copy()
        else:
            if green_tilt_deg > max_green_tilt_deg:
                print(
                    "[WARN] "
                    f"物体绿色轴倾斜{green_tilt_deg:.1f}°超过{max_green_tilt_deg:.1f}°限制"
                    "；base-z策略仍固定使用base +Z，请人工确认物体确实竖直。"
                )
            # base-z策略只将视觉绿色轴用于诊断，避免估计倾斜直接让
            # 水平接近产生额外Z分量。
            green_control = BASE_UP_AXIS.copy()

        local_up = np.array([-1.0, 0.0, 0.0])
        photo_rotation = Rotation.from_euler(
            "xyz", photo_pose[3:], degrees=False
        ).as_matrix()
        photo_forward = normalize(
            photo_rotation @ LOCAL_APPROACH_AXIS,
            "拍照姿态夹爪local +Z向前轴",
        )
        zero_rotation, zero_approach = rotation_from_axis_mapping(
            local_up, green_control, photo_forward
        )

        candidates = []
        try:
            # rotation_from_axis_mapping会把拍照前向投影到绿色轴的垂直平面。
            # 这一定义0度，不使用可能随机交换或翻转的视觉红蓝轴。
            for offset_deg in orientation_profile["candidate_offsets_deg"]:
                rotated_hint = Rotation.from_rotvec(
                    green_control * np.radians(float(offset_deg))
                ).apply(zero_approach)
                rotation, approach = rotation_from_axis_mapping(
                    local_up, green_control, rotated_hint
                )
                candidates.append(
                    build_candidate(
                        client,
                        f"front{float(offset_deg):+g}deg",
                        rotation,
                        approach,
                        object_position,
                        state.joints,
                        zero_rotation,
                        photo_pose,
                        standoff,
                        lift_height,
                    )
                )
        except Exception as exc:
            print(f"[货架正面候选构造失败] {exc}")

        print("\n========== 完整6DOF候选解算 ==========")
        print(f"模式/机械臂: {mode}/{arm}")
        print(f"标定文件: {calibration_path}")
        print(f"拍照预设: {photo_preset} ({photo_path})")
        if mode == "eye_in_hand":
            print(f"拍照瞬间末端6D(YAML): {photo_pose}")
            print(f"当前实际末端6D(仅作逆解起点): {state.pose.as_list()}")
        if args.up_axis_policy == "measured":
            print("控制轴策略: measured，视觉object +Y(绿)->gripper -X")
        else:
            print("控制轴策略: base-z，base +Z->gripper -X")
            print("视觉绿色+Y轴仅用于朝上性和倾角诊断，不直接控制夹爪倾斜")
        print("0°参考策略: 拍照姿态gripper local +Z投影到绿色轴垂直平面")
        print(f"拍照姿态夹爪向前轴(base): {photo_forward.tolist()}")
        print(f"0°接近方向(base): {zero_approach.tolist()}")
        print(
            f"候选角度配置: {orientation_profile_name} "
            f"({ORIENTATION_CONFIG_PATH})；不使用其中的货架正面向量"
        )
        print(f"候选偏角(deg): {orientation_profile['candidate_offsets_deg']}")
        print(f"绿色轴倾角(deg): {green_tilt_deg:.2f}")
        print(f"物体红色+X轴(base): {red.tolist()}")
        print(f"物体绿色+Y轴测量值(base): {green_measured.tolist()}")
        print(f"夹爪控制向上轴(base): {green_control.tolist()}")
        print(f"物体蓝色+Z轴(base): {blue.tolist()}")
        legacy.print_matrix("camera_T_object", camera_to_object)
        legacy.print_matrix("base_T_object", base_to_object)

        for item in candidates:
            print(f"\n候选 {item.name}")
            print(f"  接近方向(base): {item.approach.tolist()}")
            print(f"  目标末端欧拉角(rad): {item.grasp_pose[3:]}")
            print(f"  与0°参考姿态旋转差: {item.reference_angle_deg:.2f}°")
            print(f"  最大单关节变化: {item.max_joint_delta:.2f}°")
            print(f"  全过程关节总运动量: {item.total_joint_motion:.2f}°")
            print(f"  路径最小|J5|: {item.min_abs_j5:.2f}°")
            print(f"  评分: {item.score}")
            print(f"  结果: {'PASS' if not item.errors else 'REJECT'}")
            for error in item.errors:
                print(f"    - {error}")

        if base_errors:
            print("\n[REJECTED] 基础检查失败:")
            for error in base_errors:
                print(f"  - {error}")
            return 2
        valid = [item for item in candidates if not item.errors]
        if not valid:
            print("\n[REJECTED] 所有货架正面候选均未通过，不会发送运动命令。")
            return 2
        chosen = min(valid, key=lambda item: item.score)
        print(f"\n[SELECTED] {chosen.name}，评分={chosen.score:.2f}")
        print(f"预抓取6D: {chosen.pregrasp_pose}")
        print(f"10mm停止点6D: {chosen.near_pose}")
        print(f"抓取点6D: {chosen.grasp_pose}")

        if args.plan_only:
            print("[PLAN ONLY] 已完成只读解算，不会发送运动或夹爪命令。")
            return 0
        if arm != "left":
            print("[REJECTED] 当前夹爪执行阶段只允许已验证的左臂。")
            return 2

        confirmation = input(
            f"确认候选轴映射、空间和路径后，输入 MOVE {arm.upper()} 6D PREGRASP: "
        ).strip()
        if confirmation != f"MOVE {arm.upper()} 6D PREGRASP":
            print("已取消，没有发送运动命令。")
            return 0
        client.configure_gripper_range(0, 1000)
        client.gripper_release(speed=legacy.GRIPPER_SPEED, block=False, timeout=1)
        legacy.wait_gripper_open(client, minimum_position=990)
        client.movej_p(
            chosen.pregrasp_pose, v=legacy.SPEED_PERCENT, r=0,
            trajectory_connect=0, block=True,
        )
        actual_pregrasp = client.get_state().pose.as_list()
        error = float(np.linalg.norm(np.asarray(actual_pregrasp[:3]) - np.asarray(chosen.pregrasp_pose[:3])))
        if error > 0.015:
            raise RuntimeError(f"预抓取位置误差{error * 1000.0:.1f}mm超过15mm")

        confirmation = input(f"输入 APPROACH {arm.upper()} 6D 10MM 执行直线接近: ").strip()
        if confirmation != f"APPROACH {arm.upper()} 6D 10MM":
            print("已停在预抓取点。")
            return 0
        legacy.execute_movel_monitored(client, chosen.near_pose)
        confirmation = input(f"输入 FINAL 6D GRASP {arm.upper()} 完成最后10mm并夹取: ").strip()
        if confirmation != f"FINAL 6D GRASP {arm.upper()}":
            print("已停在抓取点前10mm。")
            return 0
        actual_grasp = legacy.execute_movel_monitored(
            client, chosen.grasp_pose,
            speed_percent=legacy.FINAL_APPROACH_SPEED_PERCENT,
            timeout_s=legacy.FINAL_APPROACH_TIMEOUT_S,
        )
        legacy.execute_gripper_pick_monitored(client)

        # 以实际抓取位姿重新生成并检查回退，避免计划值与实际值偏差。
        actual_state = client.get_state()
        actual_rotation = Rotation.from_euler("xyz", actual_grasp[3:]).as_matrix()
        actual_approach = actual_rotation @ LOCAL_APPROACH_AXIS
        actual_returns = legacy.make_return_poses(
            np.asarray(actual_grasp[:3]), actual_rotation, actual_approach,
            standoff, lift_height, photo_pose,
        )
        return_errors: list[str] = []
        legacy.validate_return_targets(actual_returns, np.asarray(actual_grasp[:3]), return_errors)
        previous = [float(value) for value in actual_state.joints]
        for stage in ("lift", "retreat", "lower", "photo"):
            solved = legacy.inverse_kinematics(client, previous, actual_returns[stage])
            delta = max(abs(float(b) - float(a)) for a, b in zip(previous, solved))
            if delta > legacy.MAX_JOINT_DELTA_DEG:
                return_errors.append(f"实际回退{stage}单关节变化{delta:.1f}°超过限制")
            if len(solved) >= 5 and abs(float(solved[4])) < MIN_J5_ABS_DEG:
                return_errors.append(f"实际回退{stage}|J5|={abs(float(solved[4])):.1f}°接近奇异")
            previous = solved
        if return_errors:
            print("[RETURN REJECTED] 机械臂保持抓取位置:")
            for error in return_errors:
                print(f"  - {error}")
            return 3
        confirmation = input(f"输入 RETURN {arm.upper()} 6D PHOTO 执行安全回退: ").strip()
        if confirmation != f"RETURN {arm.upper()} 6D PHOTO":
            print("已取消回退；机械臂保持抓取位置。")
            return 0
        legacy.execute_movel_monitored(
            client, actual_returns["lift"],
            speed_percent=legacy.RETURN_LINEAR_SPEED_PERCENT,
            timeout_s=legacy.RETURN_MOVEL_TIMEOUT_S,
        )
        legacy.execute_movel_monitored(
            client, actual_returns["retreat"],
            speed_percent=legacy.RETURN_LINEAR_SPEED_PERCENT,
            timeout_s=legacy.RETURN_MOVEL_TIMEOUT_S,
        )
        legacy.execute_movel_monitored(
            client, actual_returns["lower"],
            speed_percent=legacy.RETURN_LINEAR_SPEED_PERCENT,
            timeout_s=legacy.RETURN_MOVEL_TIMEOUT_S,
        )
        legacy.execute_movej_p_monitored(
            client, actual_returns["photo"],
            speed_percent=legacy.RETURN_MOVEJ_SPEED_PERCENT,
            timeout_s=legacy.RETURN_MOVEJ_TIMEOUT_S,
        )
        print("完整6DOF抓取与回退完成。")
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
