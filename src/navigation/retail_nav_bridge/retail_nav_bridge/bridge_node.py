"""ROS2 facade node exposing /retail_nav/v1/* APIs."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from retail_nav_msgs.action import GoToPose, GoToStation, MoveRelative, StopMapping
from retail_nav_msgs.msg import HealthSnapshot, NavEvent, NavPose, NavStatus, StationInfo
from retail_nav_msgs.srv import (
    CancelMapping,
    CancelNav,
    ClearError,
    ExportMapPackage,
    ExportVendorMapPackage,
    GetHealth,
    Hold,
    Init,
    ListStations,
    LoadMap,
    Pause,
    ReleaseHold,
    Relocate,
    Resume,
    StartMapping,
    SyncStations,
)

from retail_nav_bridge.adapters.mock import MockNavAdapter
from retail_nav_bridge.facade import FacadeEvent, NavFacade
from retail_nav_bridge.stations import load_station_catalog


def _default_stations_path() -> str:
    # Prefer share config when installed; fall back to source tree.
    here = Path(__file__).resolve()
    candidate = here.parents[1] / "config" / "retail_stations.yaml"
    if candidate.is_file():
        return str(candidate)
    return str(
        Path(get_package_share_directory("retail_nav_bridge"))
        / "config"
        / "retail_stations.yaml"
    )


def _default_vendor_map_export_dir() -> str:
    """Source-tree default requested for vendor native map exports."""
    here = Path(__file__).resolve()
    return str(here.parents[1] / "config" / "map_packages")


class RetailNavBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("retail_nav_bridge")
        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        self.declare_parameter("adapter", "mock")
        self.declare_parameter("status_hz", 10.0)
        self.declare_parameter("pose_hz", 20.0)
        self.declare_parameter("default_max_speed", 0.5)
        self.declare_parameter("default_timeout_sec", 120.0)
        self.declare_parameter("mock_linear_speed", 0.6)
        self.declare_parameter("mock_angular_speed", 0.8)
        self.declare_parameter("mock_mapping_duration_sec", 2.0)
        self.declare_parameter("stations_file", "")
        self.declare_parameter("s2_service_timeout_sec", 5.0)
        self.declare_parameter("s2_map_dir", "")
        self.declare_parameter("s2_arrive_stable_ticks", 2)
        # pose: GoToStation -> CreateTaskByPose(YAML x/y/yaw) 适合 HMI「导航节点」
        # id:  -> CreateTaskById(vendor_target_id) 适合「站台节点」
        # id_then_pose: 先 Id，失败再 Pose
        self.declare_parameter("s2_station_nav_mode", "pose")
        self.declare_parameter("s2_navigation_mode", 0)  # CreateTaskById: 0自主 1巡线
        self.declare_parameter("s2_require_server_state", False)
        self.declare_parameter(
            "vendor_map_export_dir", _default_vendor_map_export_dir()
        )
        self.declare_parameter("lh_service_timeout_sec", 5.0)
        self.declare_parameter("lh_arrive_stable_ticks", 2)
        # pose: retail_stations 坐标 -> /chassis/move_to_pose
        # marker: station_id 作为 WATER marker 名 -> /chassis/move_to_marker
        # marker_then_pose: 先 marker，失败再 pose
        self.declare_parameter("lh_station_nav_mode", "pose")
        self.declare_parameter("lh_default_floor", 1)
        self.declare_parameter("woosh_service_timeout_sec", 5.0)
        self.declare_parameter("woosh_task_type", 1)
        self.declare_parameter("woosh_allow_station_id_as_mark", False)
        self.declare_parameter("woosh_require_explicit_map_name", True)
        self.declare_parameter("woosh_allow_pose_relocation", False)

        adapter_name = self.get_parameter("adapter").get_parameter_value().string_value
        stations_file = self.get_parameter("stations_file").get_parameter_value().string_value
        if not stations_file:
            stations_file = _default_stations_path()

        catalog = load_station_catalog(stations_file)
        if adapter_name == "s2":
            from retail_nav_bridge.adapters.s2 import S2NavAdapter

            self.get_logger().info("using S2NavAdapter (server/slam/move)")
            adapter = S2NavAdapter(
                self,
                catalog=catalog,
                service_timeout_sec=float(
                    self.get_parameter("s2_service_timeout_sec").value
                ),
                map_dir=str(self.get_parameter("s2_map_dir").value),
                arrive_stable_ticks=int(
                    self.get_parameter("s2_arrive_stable_ticks").value
                ),
                station_nav_mode=str(
                    self.get_parameter("s2_station_nav_mode").value
                ),
                navigation_mode=int(
                    self.get_parameter("s2_navigation_mode").value
                ),
                require_server_state=bool(
                    self.get_parameter("s2_require_server_state").value
                ),
            )
        elif adapter_name == "lh":
            from retail_nav_bridge.adapters.lh import LhNavAdapter

            self.get_logger().info("using LhNavAdapter (lh_chassis_bridge /chassis/*)")
            adapter = LhNavAdapter(
                self,
                catalog=catalog,
                service_timeout_sec=float(
                    self.get_parameter("lh_service_timeout_sec").value
                ),
                arrive_stable_ticks=int(
                    self.get_parameter("lh_arrive_stable_ticks").value
                ),
                station_nav_mode=str(
                    self.get_parameter("lh_station_nav_mode").value
                ),
                default_floor=int(self.get_parameter("lh_default_floor").value),
            )
        elif adapter_name == "woosh":
            from retail_nav_bridge.adapters.woosh import WooshNavAdapter

            self.get_logger().info("using WooshNavAdapter (woosh_robot/ExecTask)")
            adapter = WooshNavAdapter(
                self,
                catalog=catalog,
                service_timeout_sec=float(
                    self.get_parameter("woosh_service_timeout_sec").value
                ),
                task_type=int(self.get_parameter("woosh_task_type").value),
                allow_station_id_as_mark=bool(
                    self.get_parameter("woosh_allow_station_id_as_mark").value
                ),
                require_explicit_map_name=bool(
                    self.get_parameter("woosh_require_explicit_map_name").value
                ),
                allow_pose_relocation=bool(
                    self.get_parameter("woosh_allow_pose_relocation").value
                ),
            )
        else:
            if adapter_name != "mock":
                self.get_logger().warn(
                    f"adapter='{adapter_name}' unknown; falling back to mock"
                )
            adapter = MockNavAdapter(
                linear_speed=self.get_parameter("mock_linear_speed").value,
                angular_speed=self.get_parameter("mock_angular_speed").value,
                mapping_duration_sec=self.get_parameter(
                    "mock_mapping_duration_sec"
                ).value,
            )
        self.facade = NavFacade(
            adapter,
            catalog,
            on_event=self._on_facade_event,
            stations_file=stations_file,
        )
        self._adapter_name = (
            adapter_name
            if adapter_name in ("mock", "s2", "lh", "woosh")
            else "mock"
        )
        self._stations_file = stations_file
        self._vendor_map_export_dir = str(
            self.get_parameter("vendor_map_export_dir").value
        )

        ns = "/retail_nav/v1"
        self._status_pub = self.create_publisher(NavStatus, f"{ns}/status", 10)
        self._pose_pub = self.create_publisher(NavPose, f"{ns}/pose", 10)
        self._event_pub = self.create_publisher(NavEvent, f"{ns}/event", 10)

        self.create_service(Init, f"{ns}/Init", self._srv_init, callback_group=self._cb_group)
        self.create_service(
            GetHealth, f"{ns}/GetHealth", self._srv_get_health, callback_group=self._cb_group
        )
        self.create_service(
            ClearError, f"{ns}/ClearError", self._srv_clear_error, callback_group=self._cb_group
        )
        self.create_service(
            StartMapping,
            f"{ns}/StartMapping",
            self._srv_start_mapping,
            callback_group=self._cb_group,
        )
        self.create_service(
            CancelMapping,
            f"{ns}/CancelMapping",
            self._srv_cancel_mapping,
            callback_group=self._cb_group,
        )
        self.create_service(
            LoadMap, f"{ns}/LoadMap", self._srv_load_map, callback_group=self._cb_group
        )
        self.create_service(
            Relocate, f"{ns}/Relocate", self._srv_relocate, callback_group=self._cb_group
        )
        self.create_service(
            ListStations,
            f"{ns}/ListStations",
            self._srv_list_stations,
            callback_group=self._cb_group,
        )
        self.create_service(
            SyncStations,
            f"{ns}/SyncStations",
            self._srv_sync_stations,
            callback_group=self._cb_group,
        )
        self.create_service(
            ExportMapPackage,
            f"{ns}/ExportMapPackage",
            self._srv_export_map_package,
            callback_group=self._cb_group,
        )
        self.create_service(
            ExportVendorMapPackage,
            f"{ns}/ExportVendorMapPackage",
            self._srv_export_vendor_map_package,
            callback_group=self._cb_group,
        )
        self.create_service(
            CancelNav, f"{ns}/Cancel", self._srv_cancel, callback_group=self._cb_group
        )
        self.create_service(Pause, f"{ns}/Pause", self._srv_pause, callback_group=self._cb_group)
        self.create_service(
            Resume, f"{ns}/Resume", self._srv_resume, callback_group=self._cb_group
        )
        self.create_service(Hold, f"{ns}/Hold", self._srv_hold, callback_group=self._cb_group)
        self.create_service(
            ReleaseHold,
            f"{ns}/ReleaseHold",
            self._srv_release_hold,
            callback_group=self._cb_group,
        )

        self._stop_mapping_server = ActionServer(
            self,
            StopMapping,
            f"{ns}/StopMapping",
            execute_callback=self._exec_stop_mapping,
            goal_callback=self._goal_accept,
            cancel_callback=self._cancel_accept,
            callback_group=self._cb_group,
        )
        self._goto_station_server = ActionServer(
            self,
            GoToStation,
            f"{ns}/GoToStation",
            execute_callback=self._exec_goto_station,
            goal_callback=self._goal_accept,
            cancel_callback=self._cancel_accept,
            callback_group=self._cb_group,
        )
        self._goto_pose_server = ActionServer(
            self,
            GoToPose,
            f"{ns}/GoToPose",
            execute_callback=self._exec_goto_pose,
            goal_callback=self._goal_accept,
            cancel_callback=self._cancel_accept,
            callback_group=self._cb_group,
        )
        self._move_relative_server = ActionServer(
            self,
            MoveRelative,
            f"{ns}/MoveRelative",
            execute_callback=self._exec_move_relative,
            goal_callback=self._goal_accept,
            cancel_callback=self._cancel_accept,
            callback_group=self._cb_group,
        )

        status_hz = float(self.get_parameter("status_hz").value)
        pose_hz = float(self.get_parameter("pose_hz").value)
        self.create_timer(1.0 / max(status_hz, 1.0), self._publish_status)
        self.create_timer(1.0 / max(pose_hz, 1.0), self._publish_pose)

        self.get_logger().info(
            f"retail_nav_bridge ready (adapter={self._adapter_name}, "
            f"stations={len(catalog.stations)}, file={stations_file})"
        )

    def _goal_accept(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_accept(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_facade_event(self, event: FacadeEvent) -> None:
        msg = NavEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.type = event.type
        msg.request_id = event.request_id
        msg.station_id = event.station_id
        msg.error_code = event.error_code
        msg.message = event.message
        msg.x = event.x
        msg.y = event.y
        msg.yaw = event.yaw
        self._event_pub.publish(msg)

    def _publish_status(self) -> None:
        with self._lock:
            snap = self.facade.snapshot
            msg = NavStatus()
            msg.nav_state = snap.nav_state.value
            msg.mode = snap.mode.value
            msg.request_id = snap.request_id
            msg.active_station_id = snap.active_station_id
            msg.map_id = snap.map_id
            msg.map_name = snap.map_name
            msg.mapping_status = snap.mapping_status
            msg.holding = snap.holding
            msg.localized = snap.localized
            msg.paused = snap.paused
            msg.error_code = snap.error_code
            msg.error_msg = snap.error_msg
            msg.progress = snap.progress
            self._status_pub.publish(msg)

    def _publish_pose(self) -> None:
        with self._lock:
            pose = self.facade.get_pose()
            msg = NavPose()
            msg.stamp = self.get_clock().now().to_msg()
            msg.map_id = pose.map_id
            msg.x = pose.x
            msg.y = pose.y
            msg.yaw = pose.yaw
            msg.confidence = pose.confidence
            self._pose_pub.publish(msg)

    def _fill_health(self, health) -> HealthSnapshot:
        out = HealthSnapshot()
        if health is None:
            return out
        out.runtime_ready = health.runtime_ready
        out.mapping_ready = health.mapping_ready
        out.navigation_ready = health.navigation_ready
        out.localization_ready = health.localization_ready
        out.motion_ready = health.motion_ready
        out.detail = health.detail
        return out

    def _srv_init(self, _req, res):
        with self._lock:
            result = self.facade.init()
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_get_health(self, _req, res):
        with self._lock:
            result = self.facade.get_health()
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.health = self._fill_health(result.health)
        return res

    def _srv_clear_error(self, _req, res):
        with self._lock:
            result = self.facade.clear_error()
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_start_mapping(self, req, res):
        with self._lock:
            result = self.facade.start_mapping(req.map_name)
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.session_id = result.session_id
        res.nav_state = result.nav_state
        return res

    def _srv_cancel_mapping(self, req, res):
        with self._lock:
            result = self.facade.cancel_mapping(req.reason)
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_load_map(self, req, res):
        with self._lock:
            result = self.facade.load_map(map_id=req.map_id, map_name=req.map_name)
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.map_id = result.map_id
        res.nav_state = result.nav_state
        return res

    def _srv_relocate(self, req, res):
        with self._lock:
            result = self.facade.relocate(
                station_id=req.station_id, x=req.x, y=req.y, yaw=req.yaw
            )
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        res.localized = result.localized
        return res

    def _srv_list_stations(self, _req, res):
        with self._lock:
            result = self.facade.list_stations()
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.stations = []
        for station in result.stations:
            info = StationInfo()
            info.station_id = station.station_id
            info.display_name = station.display_name
            info.x = station.x
            info.y = station.y
            info.yaw = station.yaw
            info.arrive_tol_m = station.arrive_tol_m
            info.yaw_tol_rad = station.yaw_tol_rad
            info.max_speed = station.max_speed
            info.vendor_target_id = station.vendor_target_id
            res.stations.append(info)
        return res

    def _srv_sync_stations(self, req, res):
        with self._lock:
            result = self.facade.sync_stations(
                req.package_path,
                target_stations_file=self._stations_file,
            )
            if result.ok and result.stations_file:
                self._stations_file = result.stations_file
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.station_count = result.station_count
        res.stations_file = result.stations_file
        res.map_name = result.map_name
        res.nav_state = result.nav_state
        return res

    def _srv_export_map_package(self, req, res):
        with self._lock:
            result = self.facade.export_map_package(
                req.output_dir,
                map_id=req.map_id,
                map_name=req.map_name,
                map_path=req.map_path,
                copy_map_files=bool(req.copy_map_files),
                notes=req.notes,
            )
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.package_path = result.package_path
        res.station_count = result.station_count
        res.nav_state = result.nav_state
        return res

    def _srv_export_vendor_map_package(self, req, res):
        with self._lock:
            result = self.facade.export_vendor_map_package(
                self._vendor_map_export_dir,
                map_id=req.map_id,
                map_name=req.map_name,
                notes=req.notes,
            )
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.package_path = result.package_path
        res.bmap_ok = result.bmap_ok
        res.exported_files = result.exported_files
        res.format_status = result.format_status
        res.nav_state = result.nav_state
        return res

    def _srv_cancel(self, req, res):
        with self._lock:
            result = self.facade.cancel_nav(request_id=req.request_id, reason=req.reason)
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_pause(self, req, res):
        with self._lock:
            result = self.facade.pause(req.reason)
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_resume(self, _req, res):
        with self._lock:
            result = self.facade.resume()
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_hold(self, req, res):
        with self._lock:
            result = self.facade.hold(req.reason)
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _srv_release_hold(self, _req, res):
        with self._lock:
            result = self.facade.release_hold()
        res.error_code = result.error_code
        res.error_msg = result.error_msg
        res.nav_state = result.nav_state
        return res

    def _exec_stop_mapping(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            begin = self.facade.begin_stop_mapping(goal.map_name)
        if not begin.ok:
            goal_handle.abort()
            result = StopMapping.Result()
            result.error_code = begin.error_code
            result.error_msg = begin.error_msg
            result.result_code = "error"
            return result

        rate = self.create_rate(10.0)
        final = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self.facade.cancel_mapping("action cancelled")
                goal_handle.canceled()
                result = StopMapping.Result()
                result.error_code = 7
                result.error_msg = "cancelled"
                result.result_code = "cancelled"
                return result
            with self._lock:
                tick = self.facade.tick_stop_mapping(dt=0.1)
                snap = self.facade.snapshot
            fb = StopMapping.Feedback()
            fb.progress = tick.progress
            fb.nav_state = snap.nav_state.value
            goal_handle.publish_feedback(fb)
            if tick.progress >= 1.0 or not tick.ok:
                final = tick
                break
            rate.sleep()

        result = StopMapping.Result()
        if final is None or not final.ok:
            goal_handle.abort()
            result.error_code = getattr(final, "error_code", 9)
            result.error_msg = getattr(final, "error_msg", "stop mapping failed")
            result.result_code = "error"
            return result
        goal_handle.succeed()
        result.error_code = 0
        result.error_msg = ""
        result.map_id = final.map_id
        result.map_name = final.map_name
        result.result_code = final.result_code or "done"
        return result

    def _run_nav_action(self, goal_handle, start_fn, result_cls, feedback_cls):
        with self._lock:
            begin = start_fn()
        if not begin.ok:
            goal_handle.abort()
            result = result_cls()
            result.error_code = begin.error_code
            result.error_msg = begin.error_msg
            result.result_code = "error"
            return result

        rate = self.create_rate(10.0)
        final = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self.facade.cancel_nav(reason="action cancelled")
                goal_handle.canceled()
                result = result_cls()
                result.error_code = 7
                result.error_msg = "cancelled"
                result.result_code = "cancelled"
                return result
            with self._lock:
                tick = self.facade.tick_nav(dt=0.1)
                snap = self.facade.snapshot
                pose = self.facade.get_pose()
                elapsed = self.facade.elapsed_nav_sec()
            fb = feedback_cls()
            if hasattr(fb, "distance_remaining"):
                # approximate from progress
                fb.distance_remaining = max(0.0, (1.0 - tick.progress) * 1.0)
            fb.progress = tick.progress
            fb.nav_state = snap.nav_state.value
            goal_handle.publish_feedback(fb)

            if tick.result_code in ("arrived", "blocked", "timeout", "error", "cancelled"):
                final = (tick, pose, elapsed)
                break
            if snap.nav_state.value == "ERROR":
                final = (tick, pose, elapsed)
                break
            if snap.nav_state.value == "READY" and tick.progress >= 1.0:
                final = (tick, pose, elapsed)
                break
            rate.sleep()

        result = result_cls()
        tick, pose, elapsed = final if final else (None, self.facade.get_pose(), 0.0)
        result.elapsed_sec = elapsed
        result.final_x = pose.x
        result.final_y = pose.y
        result.final_yaw = pose.yaw
        if tick is None:
            goal_handle.abort()
            result.error_code = 9
            result.error_msg = "navigation ended unexpectedly"
            result.result_code = "error"
            return result
        result.error_code = tick.error_code
        result.error_msg = tick.error_msg
        result.result_code = tick.result_code or "error"
        if tick.result_code == "arrived":
            goal_handle.succeed()
        elif tick.result_code == "cancelled":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _exec_goto_station(self, goal_handle):
        goal = goal_handle.request
        default_speed = float(self.get_parameter("default_max_speed").value)
        default_timeout = float(self.get_parameter("default_timeout_sec").value)

        def start():
            return self.facade.start_go_to_station(
                goal.station_id,
                request_id=goal.request_id,
                max_speed=goal.max_speed or default_speed,
                timeout_sec=goal.timeout_sec or default_timeout,
            )

        return self._run_nav_action(
            goal_handle, start, GoToStation.Result, GoToStation.Feedback
        )

    def _exec_goto_pose(self, goal_handle):
        goal = goal_handle.request
        default_speed = float(self.get_parameter("default_max_speed").value)
        default_timeout = float(self.get_parameter("default_timeout_sec").value)

        def start():
            return self.facade.start_go_to_pose(
                goal.x,
                goal.y,
                goal.yaw,
                request_id=goal.request_id,
                arrive_tol_m=goal.arrive_tol_m or 0.08,
                yaw_tol_rad=goal.yaw_tol_rad or 0.17,
                max_speed=goal.max_speed or default_speed,
                timeout_sec=goal.timeout_sec or default_timeout,
            )

        return self._run_nav_action(goal_handle, start, GoToPose.Result, GoToPose.Feedback)

    def _exec_move_relative(self, goal_handle):
        goal = goal_handle.request

        def start():
            return self.facade.start_move_relative(
                goal.dx,
                goal.dy,
                goal.dyaw,
                request_id=goal.request_id,
                timeout_sec=goal.timeout_sec or 30.0,
            )

        return self._run_nav_action(
            goal_handle, start, MoveRelative.Result, MoveRelative.Feedback
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = RetailNavBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
