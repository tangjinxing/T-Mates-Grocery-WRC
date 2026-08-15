"""Focused regression tests for the optional Woosh adapter."""

from __future__ import annotations

import threading
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("woosh_robot_msgs")
pytest.importorskip("woosh_task_msgs")

from woosh_robot_msgs.msg import RobotState, State as RobotStateValue

from retail_nav_bridge.adapters.base import NavGoal, Pose2D
from retail_nav_bridge.adapters.woosh import WooshNavAdapter, _NavMode
from retail_nav_bridge.stations import Station, StationCatalog


def _bare_adapter() -> WooshNavAdapter:
    """Create only the state needed to exercise pure adapter logic."""
    adapter = object.__new__(WooshNavAdapter)
    adapter._lock = threading.RLock()
    adapter._robot_state = RobotState()
    adapter._operation = None
    adapter._pose = Pose2D()
    adapter._map_id = ""
    adapter._nav_progress = 0.0
    adapter._paused = False
    adapter._holding = False
    adapter._goal_handle = None
    adapter._result_future = None
    adapter._mark_no = ""
    adapter._speed_applied = True
    return adapter


def test_woosh_idle_and_parking_are_runtime_states_not_emergency():
    adapter = _bare_adapter()

    adapter._robot_state.state.value = RobotStateValue.K_UNINIT
    assert not adapter._is_runtime_ready()

    adapter._robot_state.state.value = RobotStateValue.K_IDLE
    assert adapter._is_runtime_ready()
    assert not adapter._is_faulted()

    adapter._robot_state.state.value = RobotStateValue.K_PARKING
    assert adapter._is_runtime_ready()
    assert not adapter._is_faulted()

    adapter._robot_state.state.value = RobotStateValue.K_FAULT
    assert not adapter._is_runtime_ready()
    assert adapter._is_faulted()


def test_woosh_requires_explicit_vendor_mark_by_default():
    adapter = _bare_adapter()
    adapter._allow_station_id_as_mark = False
    adapter._catalog = StationCatalog(
        map_name="",
        stations={
            "business_station": Station(
                station_id="business_station",
                display_name="business",
                x=0.0,
                y=0.0,
                yaw=0.0,
            )
        },
    )
    goal = NavGoal(station_id="business_station")
    assert adapter._resolve_mark_no(goal) == ""

    adapter._allow_station_id_as_mark = True
    assert adapter._resolve_mark_no(goal) == "business_station"


def test_woosh_does_not_arrive_from_template_pose_before_action_result():
    class PendingResult:
        def done(self) -> bool:
            return False

    adapter = _bare_adapter()
    adapter._robot_state.state.value = RobotStateValue.K_IDLE
    adapter._goal = NavGoal(x=0.0, y=0.0, yaw=0.0, timeout_sec=60.0)
    adapter._nav_mode = _NavMode.MARK
    adapter._nav_started_at = time.monotonic()
    adapter._result_future = PendingResult()

    tick = adapter.tick_nav()

    assert tick.result_code == ""
    assert tick.progress == 0.05
    assert adapter._goal is not None
