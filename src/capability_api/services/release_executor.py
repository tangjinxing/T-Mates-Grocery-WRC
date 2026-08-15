from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import yaml

from services.motion_coordinator import MotionCoordinator
from services.motion_parameters import MotionParameters


GRASP_ROOT = Path("/home/lh/WRC/src/control/grasp_pipeline")
ARM_API_ROOT = Path("/home/lh/robot_api/arm_api_new")
ARM_IPS = {"left": "169.254.128.18", "right": "169.254.128.19"}
READY_NAME = "DELIVERY_TABLE_PLACE_READY"
MIN_J5_ABS_DEG = 5.0
MAX_JOINT_DELTA_DEG = 90.0
ARM_TIMEOUT_S = 60.0


class ReleasePlanningError(RuntimeError):
    pass


class ReleaseExecutionError(RuntimeError):
    pass


class ReleaseExecutor:
    """Plan or execute a fixed READY/TRANSITION/FINAL release chain."""

    def __init__(
        self,
        coordinator: MotionCoordinator,
        pose_config: Path,
        parameters: MotionParameters,
    ) -> None:
        self._coordinator = coordinator
        self._pose_config = pose_config
        self._parameters = parameters

    def _load_targets(self, arm: str) -> dict[str, Any]:
        if not self._pose_config.is_file():
            raise ReleasePlanningError(f"放置位姿配置不存在: {self._pose_config}")
        try:
            data = yaml.safe_load(self._pose_config.read_text(encoding="utf-8"))
        except Exception as exc:
            raise ReleasePlanningError(f"放置位姿YAML解析失败: {exc}") from exc
        poses = data.get("poses") if isinstance(data, dict) else None
        if not isinstance(poses, dict):
            raise ReleasePlanningError("放置位姿YAML缺少poses映射")

        side = arm.upper()
        arm_key = f"{arm}_arm"
        names = {
            "ready": READY_NAME,
            "transition": f"DELIVERY_TABLE_PLACE_TRANSITION_{side}",
            "final": f"DELIVERY_TABLE_PLACE_FINAL_{side}",
        }
        selected: dict[str, dict[str, Any]] = {}
        for stage, name in names.items():
            value = poses.get(name)
            if not isinstance(value, dict):
                raise ReleasePlanningError(f"缺少放置位姿: {name}")
            allowed = {"ready"} if stage == "ready" else {"candidate", "ready"}
            if value.get("status") not in allowed:
                raise ReleasePlanningError(
                    f"{name}状态={value.get('status')}，允许状态={sorted(allowed)}"
                )
            selected[stage] = value

        def finite_pose(stage: str) -> list[float]:
            value = selected[stage].get(arm_key, {}).get("pose_6d")
            if not isinstance(value, list) or len(value) != 6:
                raise ReleasePlanningError(f"{names[stage]}.{arm_key}.pose_6d必须包含6个数")
            result = [float(item) for item in value]
            if not all(math.isfinite(item) for item in result):
                raise ReleasePlanningError(f"{names[stage]}.{arm_key}.pose_6d包含非法数值")
            return result

        heights = [selected[stage].get("torso", {}).get("height_mm") for stage in selected]
        if any(value is None for value in heights):
            raise ReleasePlanningError(f"放置链躯干高度不完整: {heights}")
        normalized_heights = [int(value) for value in heights]
        if len(set(normalized_heights)) != 1:
            raise ReleasePlanningError(f"放置链躯干高度不一致: {normalized_heights}")
        return {
            "names": names,
            "ready_pose": finite_pose("ready"),
            "transition_pose": finite_pose("transition"),
            "final_pose": finite_pose("final"),
            "torso_height_mm": normalized_heights[0],
        }

    @staticmethod
    def _plan_chain(client: Any, legacy: Any, targets: dict[str, Any]) -> dict[str, Any]:
        state = client.get_state()
        robot_errors = legacy.nonzero_robot_errors(state.err)
        if robot_errors:
            raise ReleasePlanningError("机械臂错误: " + ", ".join(robot_errors))

        stages = (
            ("READY", targets["ready_pose"]),
            ("TRANSITION", targets["transition_pose"]),
            ("FINAL", targets["final_pose"]),
            ("TRANSITION_RETURN", targets["transition_pose"]),
            ("READY_RETURN", targets["ready_pose"]),
        )
        previous = [float(value) for value in state.joints]
        max_delta = 0.0
        min_abs_j5 = float("inf")
        ik: dict[str, list[float]] = {}
        for stage, pose in stages:
            errors: list[str] = []
            legacy.validate_pose_limits(stage, pose, errors)
            if errors:
                raise ReleasePlanningError("；".join(errors))
            try:
                solved = legacy.inverse_kinematics(client, previous, pose)
            except Exception as exc:
                raise ReleasePlanningError(f"{stage}逆运动学失败: {exc}") from exc
            delta = max(abs(float(b) - float(a)) for a, b in zip(previous, solved))
            if not math.isfinite(delta) or delta > MAX_JOINT_DELTA_DEG:
                raise ReleasePlanningError(
                    f"{stage}单关节最大变化{delta:.1f}°超过{MAX_JOINT_DELTA_DEG:.1f}°"
                )
            max_delta = max(max_delta, delta)
            if len(solved) < 5:
                raise ReleasePlanningError(f"{stage}逆解关节数量异常: {solved}")
            min_abs_j5 = min(min_abs_j5, abs(float(solved[4])))
            previous = solved
            ik[stage] = solved
        if min_abs_j5 < MIN_J5_ABS_DEG:
            raise ReleasePlanningError(
                f"完整放置路径最小|J5|={min_abs_j5:.1f}°低于{MIN_J5_ABS_DEG:.1f}°限制"
            )
        return {
            "max_joint_delta_deg": float(max_delta),
            "min_abs_j5_deg": float(min_abs_j5),
            "ik": ik,
        }

    def run(self, hand: str, execute: bool) -> dict[str, Any]:
        arm = hand.lower()
        if arm not in {"left", "right"}:
            raise ReleasePlanningError("hand只支持LEFT或RIGHT")
        targets = self._load_targets(arm)

        self._coordinator.acquire()
        client = None
        lift_client = None
        command_started = False
        try:
            sys.path.insert(0, str(GRASP_ROOT))
            import place_object as legacy

            sys.path.insert(0, str(ARM_API_ROOT))
            from realman_arm_api_api2 import RealmanArmClient

            client = RealmanArmClient(ip=ARM_IPS[arm], model=arm, auto_connect=False)
            client.connect()
            if arm == "left":
                lift_client = client
            else:
                lift_client = RealmanArmClient(
                    ip=ARM_IPS["left"], model="left", auto_connect=False
                )
                lift_client.connect()

            try:
                legacy.require_pose(
                    client,
                    f"{arm}臂放置READY",
                    targets["ready_pose"],
                    legacy.START_POSITION_TOLERANCE_M,
                    legacy.START_ORIENTATION_TOLERANCE_DEG,
                )
                legacy.check_lift(lift_client, targets["torso_height_mm"])
                plan = self._plan_chain(client, legacy, targets)
                if execute:
                    legacy.check_gripper_holding(client)
            except Exception as exc:
                if isinstance(exc, ReleasePlanningError):
                    raise
                raise ReleasePlanningError(str(exc)) from exc

            result: dict[str, Any] = {
                "status": "SUCCEEDED",
                "executed": bool(execute),
                "reachable": True,
                "operation": "RELEASE_AND_RETURN" if execute else "RELEASE_PLAN",
                "hand": arm.upper(),
                "torso_height_mm": targets["torso_height_mm"],
                "ready_pose": targets["ready_pose"],
                "transition_pose": targets["transition_pose"],
                "final_pose": targets["final_pose"],
                "max_joint_delta_deg": plan["max_joint_delta_deg"],
                "min_abs_j5_deg": plan["min_abs_j5_deg"],
                "message": (
                    "放置、松爪和返回READY完成"
                    if execute
                    else "只读放置规划完成，未发送运动或夹爪命令"
                ),
            }
            if not execute:
                return result

            try:
                command_started = True
                actual_transition = legacy.execute_movej_p_monitored(
                    client,
                    targets["transition_pose"],
                    self._parameters.get("release", "movej_p_speed_percent"),
                    ARM_TIMEOUT_S,
                    "MoveJ_P放置过渡",
                )
                legacy.require_pose(
                    client,
                    "放置过渡到位",
                    targets["transition_pose"],
                    legacy.ARM_POSITION_TOLERANCE_M,
                    legacy.ARM_ORIENTATION_TOLERANCE_DEG,
                )
                legacy.check_lift(lift_client, targets["torso_height_mm"])
                legacy.check_gripper_holding(client)

                actual_final = legacy.execute_movej_p_monitored(
                    client,
                    targets["final_pose"],
                    self._parameters.get("release", "movej_p_speed_percent"),
                    ARM_TIMEOUT_S,
                    "MoveJ_P最终放置",
                )
                legacy.require_pose(
                    client,
                    "最终放置到位",
                    targets["final_pose"],
                    legacy.ARM_POSITION_TOLERANCE_M,
                    legacy.ARM_ORIENTATION_TOLERANCE_DEG,
                )
                legacy.check_lift(lift_client, targets["torso_height_mm"])
                legacy.check_gripper_holding(client)
                released = legacy.release_gripper_monitored(client)

                actual_return_transition = legacy.execute_movej_p_monitored(
                    client,
                    targets["transition_pose"],
                    self._parameters.get("release", "movej_p_speed_percent"),
                    ARM_TIMEOUT_S,
                    "MoveJ_P返回过渡",
                )
                actual_ready = legacy.execute_movej_p_monitored(
                    client,
                    targets["ready_pose"],
                    self._parameters.get("release", "movej_p_speed_percent"),
                    ARM_TIMEOUT_S,
                    "MoveJ_P返回READY",
                )
                legacy.require_pose(
                    client,
                    "放置流程结束READY",
                    targets["ready_pose"],
                    legacy.ARM_POSITION_TOLERANCE_M,
                    legacy.ARM_ORIENTATION_TOLERANCE_DEG,
                )
                legacy.check_lift(lift_client, targets["torso_height_mm"])
                result.update(
                    {
                        "actual_transition_pose": actual_transition,
                        "actual_final_pose": actual_final,
                        "actual_return_transition_pose": actual_return_transition,
                        "actual_ready_pose": actual_ready,
                        "gripper_actpos": int(released.actpos),
                    }
                )
                return result
            except Exception as exc:
                try:
                    client.move_stop(block=False)
                except Exception:
                    pass
                raise ReleaseExecutionError(str(exc)) from exc
        except ReleasePlanningError:
            raise
        except ReleaseExecutionError:
            raise
        except Exception as exc:
            if command_started:
                raise ReleaseExecutionError(str(exc)) from exc
            raise ReleasePlanningError(str(exc)) from exc
        finally:
            if lift_client is not None and lift_client is not client:
                try:
                    lift_client.disconnect()
                except Exception:
                    pass
            if client is not None:
                try:
                    client.disconnect()
                except Exception:
                    pass
            self._coordinator.release()
