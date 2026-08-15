from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from services.motion_coordinator import MotionCoordinator
from services.motion_parameters import MotionParameters


GRASP_ROOT = Path("/home/lh/WRC/src/control/grasp_pipeline")


class GraspPlanningError(RuntimeError):
    pass


class GraspExecutionError(RuntimeError):
    pass


class GraspPlanner:
    """Hardware-backed, read-only grasp transform and IK planner."""

    def __init__(self, coordinator: MotionCoordinator, parameters: MotionParameters) -> None:
        self._coordinator = coordinator
        self._parameters = parameters

    @staticmethod
    def convert_input_pose(pose: list[float], rotation_order: str) -> tuple[np.ndarray, list[float]]:
        values = np.asarray(pose, dtype=float)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise GraspPlanningError("pose必须是6个有限数值")
        order = rotation_order.lower()
        if order not in {"xyz", "zyx"}:
            raise GraspPlanningError("rotation_order只支持xyz或zyx")
        rotation = Rotation.from_euler(order, values[3:], degrees=False).as_matrix()
        xyz_angles = Rotation.from_matrix(rotation).as_euler("xyz", degrees=False)
        normalized = [*values[:3].tolist(), *xyz_angles.tolist()]
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = values[:3] / 1000.0
        return transform, normalized

    def plan(
        self,
        hand: str,
        pose: list[float],
        rotation_order: str,
        execute: bool = False,
    ) -> dict[str, Any]:
        arm = hand.lower()
        if arm not in {"left", "right"}:
            raise GraspPlanningError("hand只支持LEFT或RIGHT")
        camera_to_object, normalized_pose = self.convert_input_pose(pose, rotation_order)

        self._coordinator.acquire()
        client = None
        try:
            sys.path.insert(0, str(GRASP_ROOT))
            import run_pose_target as legacy
            import run_pose_target_6d as sixd

            sys.path.insert(0, str(legacy.ARM_API_ROOT))
            from realman_arm_api_api2 import RealmanArmClient

            errors: list[str] = []
            legacy.validate_transform("camera_T_object", camera_to_object, errors)
            depth_m = float(camera_to_object[2, 3])
            if not legacy.CAMERA_DEPTH_LIMIT_M[0] <= depth_m <= legacy.CAMERA_DEPTH_LIMIT_M[1]:
                errors.append("相机深度超出允许范围")

            client = RealmanArmClient(
                ip=legacy.ARM_IPS[arm], model=arm, auto_connect=False
            )
            client.connect()
            state = client.get_state()
            robot_errors = legacy.nonzero_robot_errors(state.err)
            if robot_errors:
                errors.append("机械臂错误: " + ", ".join(robot_errors))

            capture_pose = [float(value) for value in state.pose.as_list()]
            capture_transform = legacy.euler_pose_to_transform(
                capture_pose[:3], capture_pose[3:]
            )
            gripper_to_camera, calibration_path = legacy.load_eye_in_hand(arm)
            base_to_object = capture_transform @ gripper_to_camera @ camera_to_object
            legacy.validate_transform("base_T_object", base_to_object, errors)

            profile_name, profile = sixd.load_orientation_profile("2")
            object_rotation = base_to_object[:3, :3]
            object_position = base_to_object[:3, 3]
            green = sixd.normalize(
                object_rotation[:, sixd.OBJECT_GREEN_AXIS], "物体绿色+Y轴"
            )
            upward_dot = float(green @ sixd.BASE_UP_AXIS)
            if upward_dot <= 0.0:
                errors.append("物体绿色+Y轴未朝向机械臂基座+Z")
            green_tilt = float(
                np.degrees(np.arccos(np.clip(upward_dot, -1.0, 1.0)))
            )
            max_tilt = float(profile.get("max_green_tilt_deg", 20.0))
            if green_tilt > max_tilt:
                errors.append(
                    f"物体绿色轴倾斜{green_tilt:.1f}°超过{max_tilt:.1f}°限制"
                )

            photo_rotation = Rotation.from_euler(
                "xyz", capture_pose[3:], degrees=False
            ).as_matrix()
            photo_forward = sixd.normalize(
                photo_rotation @ sixd.LOCAL_APPROACH_AXIS,
                "拍照姿态夹爪local +Z向前轴",
            )
            local_up = np.array([-1.0, 0.0, 0.0])
            zero_rotation, zero_approach = sixd.rotation_from_axis_mapping(
                local_up, green, photo_forward
            )

            candidates = []
            if not errors:
                for offset_deg in profile["candidate_offsets_deg"]:
                    rotated_hint = Rotation.from_rotvec(
                        green * np.radians(float(offset_deg))
                    ).apply(zero_approach)
                    rotation, approach = sixd.rotation_from_axis_mapping(
                        local_up, green, rotated_hint
                    )
                    candidates.append(
                        sixd.build_candidate(
                            client,
                            f"front{float(offset_deg):+g}deg",
                            rotation,
                            approach,
                            object_position,
                            state.joints,
                            zero_rotation,
                            capture_pose,
                            legacy.STANDOFF_M,
                            legacy.RETURN_LIFT_M,
                        )
                    )

            if errors:
                raise GraspPlanningError("；".join(errors))
            valid = [candidate for candidate in candidates if not candidate.errors]
            if not valid:
                reasons = []
                for candidate in candidates:
                    reasons.append(f"{candidate.name}: " + "; ".join(candidate.errors))
                raise GraspPlanningError("所有抓取候选均未通过：" + " | ".join(reasons))
            chosen = min(valid, key=lambda candidate: candidate.score)

            actual_pregrasp = None
            actual_grasp = None
            actual_return = None
            if execute:
                try:
                    gripper_state = client.get_gripper_state()
                    if int(gripper_state.error) != 0:
                        raise GraspExecutionError(
                            f"{arm}臂夹爪错误码={gripper_state.error}"
                        )
                    client.configure_gripper_range(0, 1000)
                    client.gripper_release(
                        speed=int(self._parameters.get("grasp", "gripper_speed")),
                        block=False,
                        timeout=1,
                    )
                    legacy.wait_gripper_open(client, minimum_position=990)
                    client.movej_p(
                        chosen.pregrasp_pose,
                        v=self._parameters.get(
                            "grasp", "pregrasp_movej_p_speed_percent"
                        ),
                        r=0,
                        trajectory_connect=0,
                        block=True,
                    )
                    actual_pregrasp = [
                        float(value) for value in client.get_state().pose.as_list()
                    ]
                    pregrasp_error = float(
                        np.linalg.norm(
                            np.asarray(actual_pregrasp[:3])
                            - np.asarray(chosen.pregrasp_pose[:3])
                        )
                    )
                    if pregrasp_error > 0.015:
                        raise GraspExecutionError(
                            f"预抓取位置误差{pregrasp_error * 1000.0:.1f}mm超过15mm"
                        )

                    actual_grasp = legacy.execute_movel_monitored(
                        client,
                        chosen.near_pose,
                        speed_percent=self._parameters.get(
                            "grasp", "approach_movel_speed_percent"
                        ),
                    )
                    legacy.execute_gripper_pick_monitored(
                        client,
                        speed=int(self._parameters.get("grasp", "gripper_speed")),
                        force=int(self._parameters.get("grasp", "gripper_force")),
                    )

                    # 抓取后以实际位姿重新计算、逆解并校验整条回退路径。
                    actual_state = client.get_state()
                    actual_rotation = Rotation.from_euler(
                        "xyz", actual_grasp[3:]
                    ).as_matrix()
                    actual_approach = actual_rotation @ sixd.LOCAL_APPROACH_AXIS
                    actual_returns = legacy.make_return_poses(
                        np.asarray(actual_grasp[:3]),
                        actual_rotation,
                        actual_approach,
                        legacy.STANDOFF_M - legacy.FINAL_STOP_M,
                        legacy.RETURN_LIFT_M,
                        capture_pose,
                    )
                    return_errors: list[str] = []
                    legacy.validate_return_targets(
                        actual_returns,
                        np.asarray(actual_grasp[:3]),
                        return_errors,
                    )
                    previous = [float(value) for value in actual_state.joints]
                    for stage in ("lift", "retreat", "lower", "photo"):
                        solved = legacy.inverse_kinematics(
                            client, previous, actual_returns[stage]
                        )
                        delta = max(
                            abs(float(b) - float(a))
                            for a, b in zip(previous, solved)
                        )
                        if delta > legacy.MAX_JOINT_DELTA_DEG:
                            return_errors.append(
                                f"实际回退{stage}单关节变化{delta:.1f}°超过限制"
                            )
                        if len(solved) >= 5 and abs(float(solved[4])) < sixd.MIN_J5_ABS_DEG:
                            return_errors.append(
                                f"实际回退{stage}|J5|={abs(float(solved[4])):.1f}°接近奇异"
                            )
                        previous = solved
                    if return_errors:
                        raise GraspExecutionError(
                            "抓取已完成但实际回退检查失败，机械臂保持抓取位置："
                            + "；".join(return_errors)
                        )

                    for stage in ("lift", "retreat", "lower"):
                        legacy.execute_movel_monitored(
                            client,
                            actual_returns[stage],
                            speed_percent=self._parameters.get(
                                "grasp", "return_movel_speed_percent"
                            ),
                            timeout_s=legacy.RETURN_MOVEL_TIMEOUT_S,
                        )
                    legacy.execute_movej_p_monitored(
                        client,
                        actual_returns["photo"],
                        speed_percent=self._parameters.get(
                            "grasp", "return_movej_p_speed_percent"
                        ),
                        timeout_s=legacy.RETURN_MOVEJ_TIMEOUT_S,
                    )
                    actual_return = [
                        float(value) for value in client.get_state().pose.as_list()
                    ]
                except GraspExecutionError:
                    raise
                except Exception as exc:
                    try:
                        client.move_stop(block=False)
                    except Exception:
                        pass
                    raise GraspExecutionError(str(exc)) from exc

            result = {
                "status": "SUCCEEDED",
                "executed": bool(execute),
                "reachable": True,
                "operation": "GRASP_AND_RETURN" if execute else "GRASP_PLAN",
                "hand": arm.upper(),
                "frame": "camera",
                "pose_unit": "mm_rad",
                "input_rotation_order": rotation_order.lower(),
                "normalized_rotation_order": "xyz",
                "normalized_pose": normalized_pose,
                "capture_arm_pose": capture_pose,
                "calibration_file": str(calibration_path),
                "base_T_object": base_to_object.tolist(),
                "orientation_profile": profile_name,
                "candidate_count": len(candidates),
                "valid_candidate_count": len(valid),
                "selected_candidate": chosen.name,
                "pregrasp_pose": chosen.pregrasp_pose,
                "grasp_pose": chosen.near_pose,
                "return_pose": capture_pose,
                "score": float(chosen.score),
                "max_joint_delta_deg": float(chosen.max_joint_delta),
                "min_abs_j5_deg": float(chosen.min_abs_j5),
                "green_axis_tilt_deg": green_tilt,
                "message": (
                    "抓取与安全回退完成" if execute
                    else "只读规划完成，未发送机械臂或夹爪命令"
                ),
            }
            if execute:
                result["actual_pregrasp_pose"] = actual_pregrasp
                result["actual_grasp_pose"] = actual_grasp
                result["actual_return_pose"] = actual_return
            return result
        except GraspExecutionError:
            raise
        except GraspPlanningError:
            raise
        except Exception as exc:
            raise GraspPlanningError(str(exc)) from exc
        finally:
            if client is not None:
                try:
                    client.disconnect()
                except Exception:
                    pass
            self._coordinator.release()
