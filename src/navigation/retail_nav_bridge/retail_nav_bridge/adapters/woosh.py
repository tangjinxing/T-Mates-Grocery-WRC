"""悟时 Woosh 底盘 NavAdapter — 对接 woosh_robot/ExecTask 等官方 ROS 接口.

华汇复合机器人默认底盘栈见 `chassis_ros`（`woosh_robot_msgs`）。
导航以悟时地图中的储位号 `mark_no` 为主；默认要求在站点配置中显式填写
`vendor_mark`，避免把跨底盘的业务站点名误发给底盘。
"""

from __future__ import annotations

import threading
import time
from enum import Enum
from typing import Any, Optional

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from woosh_robot_msgs.action import ExecTask
from woosh_robot_msgs.msg import (
    OperationState,
    OperationStateRobotBit,
    PoseSpeed,
    RobotState,
    Scene,
    State as RobotStateValue,
)
from woosh_robot_msgs.srv import ChangeNavMode, InitRobot, SwitchMap, Twist
from woosh_task_msgs.msg import State as TaskState

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
    MARK = "mark"


class WooshNavAdapter:
    """悟时 Woosh ROS2 adapter implementing NavAdapter."""

    def __init__(
        self,
        node: Node,
        *,
        catalog: Optional[StationCatalog] = None,
        service_timeout_sec: float = 5.0,
        exec_task_action: str = "woosh_robot/robot/ExecTask",
        pose_speed_topic: str = "woosh_robot/robot/PoseSpeed",
        operation_state_topic: str = "woosh_robot/robot/OperationState",
        robot_state_topic: str = "woosh_robot/robot/RobotState",
        scene_topic: str = "woosh_robot/robot/Scene",
        task_type: int = 1,
        allow_station_id_as_mark: bool = False,
        require_explicit_map_name: bool = True,
        allow_pose_relocation: bool = False,
    ) -> None:
        self._node = node
        self._catalog = catalog
        self._service_timeout_sec = max(service_timeout_sec, 0.5)
        self._task_type = int(task_type)
        self._allow_station_id_as_mark = bool(allow_station_id_as_mark)
        self._require_explicit_map_name = bool(require_explicit_map_name)
        self._allow_pose_relocation = bool(allow_pose_relocation)
        self._log = node.get_logger()

        self._pose = Pose2D()
        self._pose_stamp = 0.0
        self._operation: Optional[OperationState] = None
        self._robot_state: Optional[RobotState] = None
        self._holding = False
        self._paused = False
        self._map_name = catalog.map_name if catalog else ""
        self._map_id = ""
        self._map_loaded_explicitly = False
        self._localized = False

        self._nav_mode = _NavMode.NONE
        self._goal: Optional[NavGoal] = None
        self._mark_no = ""
        self._nav_started_at = 0.0
        self._nav_progress = 0.0
        self._goal_handle = None
        self._result_future = None
        self._last_task_id = 0
        self._speed_applied = False

        self._lock = threading.RLock()
        self._cb_group = ReentrantCallbackGroup()

        # Match s2 adapter QoS: pose BEST_EFFORT+VOLATILE, state RELIABLE+VOLATILE.
        qos_pose = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pose_sub = node.create_subscription(
            PoseSpeed, pose_speed_topic, self._on_pose_speed, qos_pose
        )
        self._op_sub = node.create_subscription(
            OperationState, operation_state_topic, self._on_operation_state, qos_state
        )
        self._robot_sub = node.create_subscription(
            RobotState, robot_state_topic, self._on_robot_state, qos_state
        )
        self._scene_sub = node.create_subscription(
            Scene, scene_topic, self._on_scene, qos_state
        )

        self._cli_twist = node.create_client(
            Twist, "woosh_robot/robot/Twist", callback_group=self._cb_group
        )
        self._cli_init = node.create_client(
            InitRobot, "woosh_robot/robot/InitRobot", callback_group=self._cb_group
        )
        self._cli_switch_map = node.create_client(
            SwitchMap, "woosh_robot/robot/SwitchMap", callback_group=self._cb_group
        )
        self._cli_change_nav_mode = node.create_client(
            ChangeNavMode,
            "woosh_robot/robot/ChangeNavMode",
            callback_group=self._cb_group,
        )
        self._act_exec_task = ActionClient(
            node, ExecTask, exec_task_action, callback_group=self._cb_group
        )

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

    def _apply_nav_speed(self, max_speed: float) -> None:
        """Set Woosh point-to-point max linear speed via ChangeNavMode."""
        speed = float(max_speed)
        if speed <= 0.0:
            self._speed_applied = True
            return
        req = ChangeNavMode.Request()
        req.arg.nav_mode.type.value = 2  # ArrType.K_ACCURATE
        req.arg.nav_mode.mode.value = 1  # Mode.K_AVOID
        req.arg.nav_mode.max_speed = speed
        req.arg.has_field = req.arg.NAV_MODE_FIELD_SET
        resp = self._call_service(
            self._cli_change_nav_mode,
            req,
            label="woosh_robot/robot/ChangeNavMode",
        )
        if not getattr(resp, "ok", False):
            raise RuntimeError(
                getattr(resp, "msg", "") or "ChangeNavMode failed"
            )
        self._speed_applied = True
        self._log.info(f"ChangeNavMode max_speed={speed:.3f} m/s")

    def _try_apply_nav_speed(self, max_speed: float) -> None:
        """ChangeNavMode only works after ExecTask is already navigating."""
        if self._speed_applied:
            return
        now = time.monotonic()
        last = getattr(self, "_speed_try_at", 0.0)
        if now - last < 0.4:
            return
        self._speed_try_at = now
        try:
            self._apply_nav_speed(max_speed)
        except Exception as exc:  # noqa: BLE001
            if now - self._nav_started_at > 5.0:
                self._speed_applied = True
                self._log.warn(f"ChangeNavMode gave up: {exc}")
            else:
                self._log.warn(f"ChangeNavMode not applied yet: {exc}")

    def _on_pose_speed(self, msg: PoseSpeed) -> None:
        with self._lock:
            pose_map_id = str(msg.map_id) if int(msg.map_id) else self._map_id
            self._pose = Pose2D(
                x=float(msg.pose.x),
                y=float(msg.pose.y),
                yaw=float(msg.pose.theta),
                confidence=100,
                map_id=pose_map_id,
            )
            self._pose_stamp = time.monotonic()
            self._localized = True

    def _on_operation_state(self, msg: OperationState) -> None:
        with self._lock:
            self._operation = msg

    def _on_robot_state(self, msg: RobotState) -> None:
        with self._lock:
            self._robot_state = msg

    def _on_scene(self, msg: Scene) -> None:
        with self._lock:
            if msg.map_name:
                self._map_name = str(msg.map_name)
            if int(msg.map_id):
                self._map_id = str(msg.map_id)

    def _robot_state_value(self) -> Optional[int]:
        with self._lock:
            state = self._robot_state
        return int(state.state.value) if state is not None else None

    def _is_runtime_ready(self) -> bool:
        """A published Woosh state is usable unless it is uninitialised/faulted.

        ``woosh_robot_msgs/State`` does not define an ONLINE state. In
        particular, 1 is K_UNINIT, 2 is K_IDLE and 3 is K_PARKING.
        """
        value = self._robot_state_value()
        return value not in (
            None,
            RobotStateValue.K_STATE_UNDEFINED,
            RobotStateValue.K_UNINIT,
            RobotStateValue.K_FAULT,
        )

    def _is_faulted(self) -> bool:
        return self._robot_state_value() == RobotStateValue.K_FAULT

    def _is_taskable(self) -> bool:
        with self._lock:
            op = self._operation
        if op is None:
            return False
        return (int(op.robot) & OperationStateRobotBit.K_TASKABLE) != 0

    def _resolve_mark_no(self, goal: NavGoal) -> str:
        if goal.station_id and self._catalog:
            station = self._catalog.get(goal.station_id)
            if station is not None and station.vendor_mark:
                return station.vendor_mark
            if goal.station_id and self._allow_station_id_as_mark:
                return goal.station_id
        return ""

    def _reset_nav(self) -> None:
        self._nav_mode = _NavMode.NONE
        self._goal = None
        self._mark_no = ""
        self._nav_started_at = 0.0
        self._nav_progress = 0.0
        self._goal_handle = None
        self._result_future = None
        self._paused = False
        self._speed_applied = False
        self._speed_try_at = 0.0

    def _on_action_feedback(self, _feedback_msg: Any) -> None:
        with self._lock:
            self._nav_progress = min(0.95, max(self._nav_progress, 0.15))

    def get_health(self) -> HealthInfo:
        """Health flags aligned with S2NavAdapter semantics.

        ``localization_ready`` means the relocate/init service is available, not
        that PoseSpeed is continuously fresh (same idea as S2 + slam/Relocation).
        """
        with self._lock:
            robot_state = self._robot_state
            has_pose = self._pose.confidence > 0 or self._localized
        mapping_ready = False
        navigation_ready = self._act_exec_task.server_is_ready()
        localization_ready = self._cli_init.service_is_ready()
        motion_ready = (not self._holding) and (
            self._cli_twist.service_is_ready() or navigation_ready
        )
        detail_parts: list[str] = []
        if robot_state is None:
            detail_parts.append("no RobotState yet")
            # Fallback like S2 when vendor state topic is missing.
            runtime = bool(has_pose and (navigation_ready or localization_ready))
            if runtime:
                detail_parts.append("runtime via PoseSpeed fallback")
        else:
            if self._is_faulted():
                runtime = False
                detail_parts.append("robot fault")
            elif not self._is_runtime_ready():
                runtime = False
                detail_parts.append("robot uninitialized")
            else:
                runtime = True
        if self._holding:
            detail_parts.append("holding")
        if not self._is_taskable():
            detail_parts.append("robot not taskable")
        return HealthInfo(
            runtime_ready=runtime,
            mapping_ready=mapping_ready,
            navigation_ready=navigation_ready,
            localization_ready=localization_ready,
            motion_ready=motion_ready,
            detail="; ".join(detail_parts) or "woosh adapter",
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
            error_msg="mapping not supported on woosh chassis (use vendor map tools)",
        )

    def tick_stop_mapping(self, map_name: str = "", dt: float = 0.1) -> AdapterResult:
        del map_name, dt
        return AdapterResult(
            error_code=ErrorCode.INVALID_STATE,
            error_msg="mapping not supported on woosh chassis",
            result_code=ResultCode.ERROR.value,
        )

    def cancel_mapping(self, reason: str = "") -> AdapterResult:
        return AdapterResult(error_msg=reason)

    def load_map(self, map_id: str = "", map_name: str = "") -> AdapterResult:
        name = map_name or map_id
        if not name and not self._require_explicit_map_name:
            name = self._map_name
        if not name:
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg="explicit map_name required for woosh SwitchMap",
            )
        req = SwitchMap.Request()
        req.arg.map_name = str(name)
        try:
            resp = self._call_service(
                self._cli_switch_map, req, label="woosh_robot/robot/SwitchMap"
            )
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if not resp.ok:
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=getattr(resp, "msg", "") or "SwitchMap failed",
            )
        self._map_name = name
        self._map_loaded_explicitly = True
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
        if self._require_explicit_map_name and not self._map_loaded_explicitly:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="LoadMap with an explicit Woosh map_name is required first",
            )
        req = InitRobot.Request()
        if self._allow_pose_relocation:
            if station is not None:
                x, y, yaw = station.x, station.y, station.yaw
            req.arg.is_record = False
            req.arg.pose.x = float(x)
            req.arg.pose.y = float(y)
            req.arg.pose.theta = float(yaw)
        elif station is not None or any(value != 0.0 for value in (x, y, yaw)):
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg=(
                    "pose relocation is disabled for woosh; use Relocate {} "
                    "to record the current pose, or enable "
                    "woosh_allow_pose_relocation only with calibrated Woosh coordinates"
                ),
            )
        else:
            # The vendor-supported safe initialisation mode records the
            # robot's current pose rather than applying copied coordinates.
            req.arg.is_record = True
        try:
            resp = self._call_service(
                self._cli_init, req, label="woosh_robot/robot/InitRobot"
            )
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if not resp.ok:
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=getattr(resp, "msg", "") or "InitRobot failed",
            )
        self._localized = True
        with self._lock:
            if self._allow_pose_relocation:
                self._pose = Pose2D(
                    x=x, y=y, yaw=yaw, confidence=100, map_id=self._map_id
                )
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
                error_msg="relative motion not supported on woosh adapter",
            )

        if self._require_explicit_map_name and not self._map_loaded_explicitly:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="LoadMap with an explicit Woosh map_name is required first",
            )

        if not self._is_runtime_ready():
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="woosh robot is uninitialized or faulted",
            )
        if not self._is_taskable():
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="woosh robot is not taskable",
            )

        mark_no = self._resolve_mark_no(goal)
        if not mark_no:
            return AdapterResult(
                error_code=ErrorCode.UNKNOWN_STATION,
                error_msg=(
                    "woosh navigation requires a configured vendor_mark; "
                    "set woosh_allow_station_id_as_mark:=true only when "
                    "station_id exactly matches the Woosh mark_no"
                ),
            )

        self._reset_nav()
        self._goal = goal
        self._mark_no = mark_no
        self._nav_started_at = time.monotonic()

        if not self._act_exec_task.wait_for_server(
            timeout_sec=self._service_timeout_sec
        ):
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="woosh_robot/robot/ExecTask not available",
            )

        with self._lock:
            self._last_task_id = max(self._last_task_id + 1, int(time.time()))
            task_id = self._last_task_id
        msg = ExecTask.Goal()
        msg.arg.task_id = task_id
        msg.arg.type.value = self._task_type
        msg.arg.direction.value = 0
        msg.arg.task_type_no = 0
        msg.arg.mark_no = mark_no

        send_future = self._act_exec_task.send_goal_async(
            msg, feedback_callback=self._on_action_feedback
        )
        event = threading.Event()
        send_future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._service_timeout_sec):
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg="ExecTask accept timeout",
            )
        try:
            handle = send_future.result()
        except Exception as exc:  # noqa: BLE001
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=f"ExecTask goal submission failed: {exc}",
            )
        if handle is None or not handle.accepted:
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg="ExecTask goal rejected",
            )
        self._nav_mode = _NavMode.MARK
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        self._log.info(f"ExecTask accepted mark_no='{mark_no}' task_id={task_id}")
        self._try_apply_nav_speed(goal.max_speed)
        return AdapterResult(session_id=str(task_id))

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
        self._try_apply_nav_speed(goal.max_speed)
        elapsed = time.monotonic() - self._nav_started_at
        if goal.timeout_sec > 0 and elapsed > goal.timeout_sec:
            self.cancel_nav("navigation timeout")
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg="navigation timeout",
                result_code=ResultCode.TIMEOUT.value,
                pose=self.get_pose(),
            )

        if self._is_faulted():
            self.cancel_nav("woosh fault")
            return AdapterResult(
                error_code=ErrorCode.SAFETY,
                error_msg="woosh chassis fault",
                result_code=ResultCode.ERROR.value,
                pose=self.get_pose(),
            )

        if self._result_future is not None and self._result_future.done():
            try:
                wrapped = self._result_future.result()
                ret = wrapped.result.ret
                state_val = int(ret.state.value)
            except Exception as exc:  # noqa: BLE001
                self._reset_nav()
                return AdapterResult(
                    error_code=ErrorCode.VENDOR,
                    error_msg=f"ExecTask result failed: {exc}",
                    result_code=ResultCode.ERROR.value,
                    pose=self.get_pose(),
                )
            if state_val == int(TaskState.K_COMPLETED):
                self._reset_nav()
                return AdapterResult(
                    progress=1.0,
                    result_code=ResultCode.ARRIVED.value,
                    pose=self.get_pose(),
                )
            self._reset_nav()
            msg = getattr(ret, "msg", "") or "navigation failed"
            if state_val == int(TaskState.K_CANCELED):
                msg = "navigation canceled"
            return AdapterResult(
                error_code=(
                    ErrorCode.CANCELLED
                    if state_val == int(TaskState.K_CANCELED)
                    else ErrorCode.VENDOR
                ),
                error_msg=msg,
                result_code=(
                    ResultCode.CANCELLED.value
                    if state_val == int(TaskState.K_CANCELED)
                    else ResultCode.ERROR.value
                ),
                pose=self.get_pose(),
            )

        # Pose coordinates can belong to a different map or be stale.  The
        # current ExecTask action result is the only authoritative completion
        # signal for a native mark navigation task.
        with self._lock:
            self._nav_progress = max(self._nav_progress, 0.05)
        return AdapterResult(
            progress=self._nav_progress, pose=self.get_pose(), result_code=""
        )

    def cancel_nav(self, reason: str = "") -> AdapterResult:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        try:
            if self._cli_twist.service_is_ready():
                req = Twist.Request()
                req.arg.linear = 0.0
                req.arg.angular = 0.0
                self._call_service(
                    self._cli_twist, req, label="woosh_robot/robot/Twist"
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
        saved_mark = self._mark_no
        self.cancel_nav("pause")
        self._goal = saved_goal
        self._mark_no = saved_mark
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
