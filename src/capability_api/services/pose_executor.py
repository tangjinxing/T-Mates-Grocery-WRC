from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

from services.motion_coordinator import MotionCoordinator
from services.motion_parameters import MotionParameters


CONTROL_ROOT = Path("/home/lh/WRC/src/control")
GRASP_PIPELINE_ROOT = CONTROL_ROOT / "grasp_pipeline"
CONSOLE_ROOT = CONTROL_ROOT / "initial_pose_console"


class PoseExecutor:
    """Run one complete preset through the already verified preset mover."""

    def __init__(self, coordinator: MotionCoordinator, parameters: MotionParameters) -> None:
        self._coordinator = coordinator
        self._parameters = parameters

    def execute(self, resolved: dict[str, Any]) -> None:
        self._coordinator.acquire()
        robot = None
        try:
            sys.path.insert(0, str(GRASP_PIPELINE_ROOT))
            sys.path.insert(0, str(CONSOLE_ROOT))
            from acquire_sample import ROBOT_CONFIG_PATH, load_yaml
            from hardware.robot_manager import RobotManager
            from run_pose_target import inverse_kinematics, nonzero_robot_errors

            targets = resolved["targets"]
            applied = resolved["apply_components"]
            preset: dict[str, Any] = {
                "torso": {"height": targets.get("torso", {}).get("height_mm")},
                "head": targets.get("head", {"yaw": None, "pitch": None}),
                "left_arm": targets.get("left_arm", {"pose_6d": None}),
                "right_arm": targets.get("right_arm", {"pose_6d": None}),
            }
            config = load_yaml(ROBOT_CONFIG_PATH)
            robot = RobotManager(config)
            required_arms: set[str] = set()
            if "torso" in applied or "left_arm" in applied:
                required_arms.add("left")
            if "right_arm" in applied:
                required_arms.add("right")
            for side in ("left", "right"):
                if side in required_arms:
                    robot.connect_arm(side)
            if "head" in applied:
                robot.connect_head()

            # 整组预检必须在第一条运动命令之前全部完成。
            if "torso" in applied:
                height = int(preset["torso"]["height"])
                lift = config["lift"]
                if not int(lift["min_height"]) <= height <= int(lift["max_height"]):
                    raise RuntimeError(f"躯干目标{height}mm超出安全范围")
                current_height = robot.get_actual_lift_height()
                if not int(lift["min_height"]) <= current_height <= int(lift["max_height"]):
                    raise RuntimeError(f"躯干当前反馈{current_height}mm异常")
            if "head" in applied:
                yaw, pitch = int(preset["head"]["yaw"]), int(preset["head"]["pitch"])
                yaw_limits = config["head"]["yaw_limits"]
                pitch_limits = config["head"]["pitch_limits"]
                if not int(yaw_limits[0]) <= yaw <= int(yaw_limits[1]):
                    raise RuntimeError(f"头部yaw={yaw}超出安全范围")
                if not int(pitch_limits[0]) <= pitch <= int(pitch_limits[1]):
                    raise RuntimeError(f"头部pitch={pitch}超出安全范围")
                robot.get_actual_head()
            for side, component in (("left", "left_arm"), ("right", "right_arm")):
                if component not in applied:
                    continue
                target = [float(value) for value in preset[component]["pose_6d"]]
                robot._validate_pose(target)
                client = robot._arms[side]
                state = client.get_state()
                errors = nonzero_robot_errors(state.err)
                if errors:
                    raise RuntimeError(f"{side}臂存在错误: {', '.join(errors)}")
                solved = inverse_kinematics(client, state.joints, target)
                delta = max(abs(float(b) - float(a)) for a, b in zip(state.joints, solved))
                if not math.isfinite(delta) or delta > 90.0:
                    raise RuntimeError(f"{side}臂目标逆解单关节最大变化{delta:.1f}°，超过90.0°")

            # 与已验证采集流程保持同一顺序：躯干、头部、左臂、右臂。
            if "torso" in applied:
                target = int(preset["torso"]["height"])
                actual = robot.get_actual_lift_height()
                tolerance = int(config["lift"].get("arrival_tolerance", 10))
                if abs(actual - target) > tolerance:
                    robot.move_lift(
                        target,
                        self._parameters.get("pose_prepare", "lift_speed"),
                        confirmed=True,
                    )
            if "head" in applied:
                robot.move_head(
                    int(preset["head"]["yaw"]), int(preset["head"]["pitch"]), confirmed=True
                )
            for side, component in (("left", "left_arm"), ("right", "right_arm")):
                if component in applied:
                    robot.move_arm_pose(
                        side, [float(value) for value in preset[component]["pose_6d"]],
                        self._parameters.get(
                            "pose_prepare", "arm_movej_p_speed_percent"
                        ),
                        confirmed=True,
                    )
        finally:
            if robot is not None:
                robot.close()
            self._coordinator.release()
