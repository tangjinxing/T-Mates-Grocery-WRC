"""华汇 / 云迹 WATER 底盘 NavAdapter — 对接 lh_chassis_bridge (/chassis/*).

前提：同一 ROS 域内已启动 `ros2 launch lh_chassis_bridge water_bridge.launch.py`。
接口定义见华汇工程 `chassis_water/lh_chassis_interfaces` 与 `lh_chassis_bridge`。
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum
from typing import Any, Optional

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from lh_chassis_interfaces.action import MoveToMarker, MoveToPose
from lh_chassis_interfaces.msg import OperationState, PoseSpeed
from lh_chassis_interfaces.srv import PositionAdjustPose, SetCurrentMap

from retail_nav_bridge.adapters.base import (
    AdapterResult,
    HealthInfo,
    NavGoal,
    Pose2D,
)
from retail_nav_bridge.constants import ErrorCode, ResultCode
from retail_nav_bridge.stations import Station, StationCatalog


class _NavMode(str, Enum):
    NONE = "none"
    POSE = "pose"
    MARKER = "marker"


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class LhNavAdapter:
    """WATER 底盘 ROS2 adapter implementing NavAdapter."""

    def __init__(
        self,
        node: Node,
        *,
        catalog: Optional[StationCatalog] = None,
        service_timeout_sec: float = 5.0,
        arrive_stable_ticks: int = 2,
        station_nav_mode: str = "pose",
        default_floor: int = 1,
        move_to_pose_action: str = "/chassis/move_to_pose",
        move_to_marker_action: str = "/chassis/move_to_marker",
        pose_speed_topic: str = "/chassis/pose_speed",
        operation_state_topic: str = "/chassis/operation_state",
        stop_service: str = "/chassis/stop",
    ) -> None:
        self._node = node
        self._catalog = catalog
        self._service_timeout_sec = max(service_timeout_sec, 0.5)
        self._arrive_stable_ticks = max(arrive_stable_ticks, 1)
        mode = str(station_nav_mode or "pose").strip().lower()
        if mode not in ("pose", "marker", "marker_then_pose"):
            mode = "pose"
        self._station_nav_mode = mode
        self._default_floor = int(default_floor)
        self._log = node.get_logger()

        self._pose = Pose2D()
        self._pose_stamp = 0.0
        self._operation: Optional[OperationState] = None
        self._holding = False
        self._paused = False
        self._map_name = catalog.map_name if catalog else ""
        self._map_id = ""
        self._localized = False

        self._nav_mode = _NavMode.NONE
        self._goal: Optional[NavGoal] = None
        self._nav_started_at = 0.0
        self._arrive_hits = 0
        self._nav_progress = 0.0
        self._goal_handle = None
        self._result_future = None
        self._last_move_status = ""

        self._lock = threading.RLock()

        self._pose_sub = node.create_subscription(
            PoseSpeed, pose_speed_topic, self._on_pose_speed, 10
        )
        self._op_sub = node.create_subscription(
            OperationState, operation_state_topic, self._on_operation_state, 10
        )

        self._cli_stop = node.create_client(Trigger, stop_service)
        self._cli_set_map = node.create_client(SetCurrentMap, "/chassis/set_current_map")
        self._cli_reloc = node.create_client(
            PositionAdjustPose, "/chassis/position_adjust_pose"
        )

        self._act_pose = ActionClient(node, MoveToPose, move_to_pose_action)
        self._act_marker = ActionClient(node, MoveToMarker, move_to_marker_action)

    def _call_service(self, client: Any, request: Any, *, label: str) -> Any:
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            raise TimeoutError(f"{label} not available")
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._service_timeout_sec):
            raise TimeoutError(f"{label} timed out")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def _on_pose_speed(self, msg: PoseSpeed) -> None:
        with self._lock:
            self._pose = Pose2D(
                x=float(msg.x),
                y=float(msg.y),
                yaw=float(msg.theta),
                confidence=100,
                map_id=self._map_id,
            )
            self._pose_stamp = time.monotonic()
            self._localized = True

    def _on_operation_state(self, msg: OperationState) -> None:
        with self._lock:
            self._operation = msg

    def _reset_nav(self) -> None:
        self._nav_mode = _NavMode.NONE
        self._goal = None
        self._nav_started_at = 0.0
        self._arrive_hits = 0
        self._nav_progress = 0.0
        self._goal_handle = None
        self._result_future = None
        self._last_move_status = ""
        self._paused = False

    def _on_action_feedback(self, feedback_msg: Any) -> None:
        fb = feedback_msg.feedback
        status = str(getattr(fb, "move_status", "") or "")
        if status:
            self._last_move_status = status
        if status == "running":
            self._nav_progress = min(0.95, max(self._nav_progress, 0.1))

    def _fill_move_goal_pose(self, goal: NavGoal, msg: MoveToPose.Goal) -> None:
        msg.x = float(goal.x)
        msg.y = float(goal.y)
        msg.theta = float(goal.yaw)
        msg.distance_tolerance = float(goal.arrive_tol_m)
        msg.theta_tolerance = float(goal.yaw_tol_rad)
        msg.timeout_s = float(goal.timeout_sec or 120.0)
        msg.yaw_goal_reverse_allowed = 0

    def _fill_move_goal_marker(self, goal: NavGoal, msg: MoveToMarker.Goal) -> None:
        msg.marker = str(goal.station_id or "")
        msg.distance_tolerance = float(goal.arrive_tol_m)
        msg.theta_tolerance = float(goal.yaw_tol_rad)
        msg.timeout_s = float(goal.timeout_sec or 120.0)
        msg.yaw_goal_reverse_allowed = 0

    def _send_action(
        self, client: ActionClient, goal_msg: Any, *, label: str
    ) -> AdapterResult:
        if not client.wait_for_server(timeout_sec=self._service_timeout_sec):
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg=f"{label} action server not available",
            )
        send_future = client.send_goal_async(
            goal_msg, feedback_callback=self._on_action_feedback
        )
        event = threading.Event()
        send_future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._service_timeout_sec):
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg=f"{label} accept timeout",
            )
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=f"{label} goal rejected",
            )
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        return AdapterResult()

    def get_health(self) -> HealthInfo:
        with self._lock:
            op = self._operation
            pose_fresh = (time.monotonic() - self._pose_stamp) < 5.0
        nav_ready = self._act_pose.server_is_ready() or self._act_marker.server_is_ready()
        localization_ready = self._cli_reloc.service_is_ready() and (
            pose_fresh or self._localized
        )
        runtime = True
        detail_parts: list[str] = []
        if op is not None:
            if op.estop:
                runtime = False
                detail_parts.append("estop")
            if not op.online:
                runtime = False
                detail_parts.append("offline")
        if not pose_fresh and not self._localized:
            detail_parts.append("no pose_speed yet")
        if self._holding:
            detail_parts.append("holding")
        motion_ready = (not self._holding) and self._cli_stop.service_is_ready()
        return HealthInfo(
            runtime_ready=runtime and (pose_fresh or op is not None),
            mapping_ready=True,
            navigation_ready=nav_ready,
            localization_ready=localization_ready,
            motion_ready=motion_ready,
            detail="; ".join(detail_parts) or "lh adapter",
        )

    def get_pose(self) -> Pose2D:
        with self._lock:
            return Pose2D(
                x=self._pose.x,
                y=self._pose.y,
                yaw=self._pose.yaw,
                confidence=self._pose.confidence,
                map_id=self._pose.map_id or self._map_id,
            )

    def start_mapping(self, map_name: str = "") -> AdapterResult:
        del map_name
        return AdapterResult(
            error_code=ErrorCode.INVALID_STATE,
            error_msg="mapping not supported on lh chassis (use vendor map tools)",
        )

    def tick_stop_mapping(self, map_name: str = "", dt: float = 0.1) -> AdapterResult:
        del map_name, dt
        return AdapterResult(
            error_code=ErrorCode.INVALID_STATE,
            error_msg="mapping not supported on lh chassis",
            result_code=ResultCode.ERROR.value,
        )

    def cancel_mapping(self, reason: str = "") -> AdapterResult:
        return AdapterResult(error_msg=reason)

    def load_map(self, map_id: str = "", map_name: str = "") -> AdapterResult:
        name = map_name or map_id or self._map_name
        if not name:
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg="map_name required for lh SetCurrentMap",
            )
        req = SetCurrentMap.Request()
        req.map_name = str(name)
        req.floor = self._default_floor
        try:
            resp = self._call_service(
                self._cli_set_map, req, label="/chassis/set_current_map"
            )
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if not resp.success:
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=resp.message or "SetCurrentMap failed",
            )
        self._map_name = name
        if map_id:
            self._map_id = map_id
        return AdapterResult(map_id=self._map_id, map_name=self._map_name)

    def relocate(
        self,
        *,
        station: Optional[Station] = None,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> AdapterResult:
        if station is not None:
            x, y, yaw = station.x, station.y, station.yaw
        req = PositionAdjustPose.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(yaw)
        req.floor = self._default_floor
        try:
            resp = self._call_service(
                self._cli_reloc, req, label="/chassis/position_adjust_pose"
            )
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if not resp.success:
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=resp.message or "position adjust failed",
            )
        self._localized = True
        with self._lock:
            self._pose = Pose2D(
                x=x, y=y, yaw=yaw, confidence=100, map_id=self._map_id
            )
            self._pose_stamp = time.monotonic()
        return AdapterResult(pose=self.get_pose())

    def start_nav(self, goal: NavGoal) -> AdapterResult:
        if self._holding:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="holding chassis",
            )
        if goal.relative:
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="relative motion not supported on lh chassis",
            )

        self._reset_nav()
        self._goal = goal
        self._nav_started_at = time.monotonic()

        if goal.station_id:
            return self._start_station(goal)
        return self._start_pose(goal)

    def _start_station(self, goal: NavGoal) -> AdapterResult:
        mode = self._station_nav_mode
        if mode == "pose":
            self._log.info(
                f"station '{goal.station_id}' -> /chassis/move_to_pose "
                f"({goal.x:.3f},{goal.y:.3f},{goal.yaw:.3f})"
            )
            return self._start_pose(goal)
        if mode == "marker":
            return self._start_marker(goal)
        result = self._start_marker(goal)
        if result.ok:
            return result
        self._log.warn(
            f"move_to_marker failed ({result.error_msg}); fallback move_to_pose"
        )
        self._reset_nav()
        self._goal = goal
        self._nav_started_at = time.monotonic()
        return self._start_pose(goal)

    def _start_pose(self, goal: NavGoal) -> AdapterResult:
        msg = MoveToPose.Goal()
        self._fill_move_goal_pose(goal, msg)
        self._nav_mode = _NavMode.POSE
        result = self._send_action(self._act_pose, msg, label="move_to_pose")
        if result.ok:
            self._log.info(
                f"move_to_pose accepted ({goal.x:.3f},{goal.y:.3f},yaw={goal.yaw:.3f})"
            )
        return result

    def _start_marker(self, goal: NavGoal) -> AdapterResult:
        if not goal.station_id:
            return AdapterResult(
                error_code=ErrorCode.UNKNOWN_STATION,
                error_msg="station_id required for marker navigation",
            )
        msg = MoveToMarker.Goal()
        self._fill_move_goal_marker(goal, msg)
        self._nav_mode = _NavMode.MARKER
        result = self._send_action(self._act_marker, msg, label="move_to_marker")
        if result.ok:
            self._log.info(f"move_to_marker accepted marker='{goal.station_id}'")
        return result

    def tick_nav(self, dt: float = 0.1) -> AdapterResult:
        del dt
        if self._paused:
            return AdapterResult(progress=self._nav_progress, pose=self.get_pose())
        if self._goal is None or self._nav_mode == _NavMode.NONE:
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="no active goal",
                result_code=ResultCode.ERROR.value,
            )

        goal = self._goal
        elapsed = time.monotonic() - self._nav_started_at
        if goal.timeout_sec > 0 and elapsed > goal.timeout_sec:
            self.cancel_nav("navigation timeout")
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg="navigation timeout",
                result_code=ResultCode.TIMEOUT.value,
                pose=self.get_pose(),
            )

        with self._lock:
            op = self._operation
        if op is not None and op.estop:
            self.cancel_nav("estop")
            return AdapterResult(
                error_code=ErrorCode.SAFETY,
                error_msg="estop",
                result_code=ResultCode.ERROR.value,
                pose=self.get_pose(),
            )

        if self._result_future is not None and self._result_future.done():
            wrapped = self._result_future.result()
            result = wrapped.result
            if not result.success:
                self._reset_nav()
                return AdapterResult(
                    error_code=ErrorCode.VENDOR,
                    error_msg=result.message or "navigation failed",
                    result_code=ResultCode.ERROR.value,
                    pose=self.get_pose(),
                )
            self._reset_nav()
            return AdapterResult(
                progress=1.0,
                result_code=ResultCode.ARRIVED.value,
                pose=self.get_pose(),
            )

        pose = self.get_pose()
        dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
        dyaw = abs(_wrap_angle(goal.yaw - pose.yaw))
        geo_ok = dist <= goal.arrive_tol_m and dyaw <= goal.yaw_tol_rad
        navigating = op.navigating if op is not None else self._last_move_status == "running"
        if self._last_move_status == "succeeded" or (geo_ok and not navigating):
            self._arrive_hits += 1
        else:
            self._arrive_hits = 0

        if self._arrive_hits >= self._arrive_stable_ticks:
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
            self._reset_nav()
            return AdapterResult(
                progress=1.0,
                result_code=ResultCode.ARRIVED.value,
                pose=pose,
            )

        approx = max(0.0, 1.0 - dist / max(dist + 1.0, 1e-3))
        self._nav_progress = max(self._nav_progress, min(0.95, approx))
        return AdapterResult(
            progress=self._nav_progress, pose=pose, result_code=""
        )

    def cancel_nav(self, reason: str = "") -> AdapterResult:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        try:
            if self._cli_stop.service_is_ready():
                self._call_service(
                    self._cli_stop, Trigger.Request(), label="/chassis/stop"
                )
        except Exception:  # noqa: BLE001
            pass
        self._reset_nav()
        return AdapterResult(
            result_code=ResultCode.CANCELLED.value,
            error_msg=reason,
            pose=self.get_pose(),
        )

    def pause_nav(self) -> AdapterResult:
        saved_goal = self._goal
        self.cancel_nav("pause")
        self._goal = saved_goal
        self._paused = True
        self._nav_mode = _NavMode.NONE
        return AdapterResult(pose=self.get_pose())

    def resume_nav(self) -> AdapterResult:
        if not self._paused:
            return AdapterResult(pose=self.get_pose())
        goal = self._goal
        self._paused = False
        if goal is None:
            return AdapterResult(pose=self.get_pose())
        return self.start_nav(goal)

    def hold(self, reason: str = "") -> AdapterResult:
        if self._nav_mode != _NavMode.NONE:
            self.cancel_nav(reason or "hold")
        self._holding = True
        self._paused = True
        return AdapterResult(error_msg=reason, pose=self.get_pose())

    def release_hold(self) -> AdapterResult:
        self._holding = False
        self._paused = False
        return AdapterResult(pose=self.get_pose())
