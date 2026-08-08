"""Unit tests for state machine + mock facade (no ROS required)."""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from retail_nav_bridge.adapters.base import HealthInfo
from retail_nav_bridge.adapters.mock import MockNavAdapter
from retail_nav_bridge.constants import ErrorCode, EventType, NavState
from retail_nav_bridge.facade import NavFacade
from retail_nav_bridge.state_machine import NavStateMachine
from retail_nav_bridge.stations import load_station_catalog

STATIONS = Path(__file__).resolve().parents[1] / "config" / "retail_stations.yaml"


@pytest.fixture
def facade() -> NavFacade:
    catalog = load_station_catalog(STATIONS)
    adapter = MockNavAdapter(
        linear_speed=2.0,
        angular_speed=3.0,
        mapping_duration_sec=0.3,
    )
    return NavFacade(adapter, catalog)


def test_load_stations():
    catalog = load_station_catalog(STATIONS)
    assert "delivery_desk" in catalog.stations
    assert "judge_zone_1" in catalog.stations
    assert catalog.get("missing") is None


def test_illegal_start_mapping_from_unready():
    sm = NavStateMachine()
    assert sm.state == NavState.UNREADY
    ok, code, _ = sm.transition(NavState.MAPPING)
    assert not ok
    assert code == ErrorCode.INVALID_STATE


def test_navigation_readiness_does_not_require_mapping_capability():
    health = HealthInfo(
        runtime_ready=True,
        mapping_ready=False,
        navigation_ready=True,
        localization_ready=True,
        motion_ready=True,
    )
    assert health.ready


def test_init_then_mapping_save(facade: NavFacade):
    events = []
    facade.on_event = events.append

    assert facade.init().ok
    assert facade.snapshot.nav_state == NavState.IDLE

    assert facade.start_mapping("demo").ok
    assert facade.snapshot.nav_state == NavState.MAPPING
    assert facade.begin_stop_mapping("demo").ok

    for _ in range(50):
        result = facade.tick_stop_mapping(dt=0.1)
        if result.progress >= 1.0:
            break
        time.sleep(0.05)
    assert facade.snapshot.nav_state == NavState.IDLE
    assert any(e.type == EventType.MAPPING_DONE.value for e in events)


def test_goto_station_arrived(facade: NavFacade):
    events = []
    facade.on_event = events.append
    assert facade.init().ok
    assert facade.load_map(map_name="retail_default").ok
    assert facade.relocate(station_id="start").ok
    assert facade.snapshot.nav_state == NavState.READY

    assert facade.start_go_to_station("delivery_desk", timeout_sec=30.0).ok
    assert facade.snapshot.nav_state == NavState.NAVIGATING

    for _ in range(200):
        tick = facade.tick_nav(dt=0.1)
        if tick.result_code == "arrived":
            break
        time.sleep(0.01)

    assert facade.snapshot.nav_state == NavState.READY
    assert any(e.type == EventType.ARRIVED.value for e in events)
    pose = facade.get_pose()
    station = facade.catalog.get("delivery_desk")
    assert abs(pose.x - station.x) < 0.1
    assert abs(pose.y - station.y) < 0.1


def test_unknown_station(facade: NavFacade):
    assert facade.init().ok
    assert facade.relocate(station_id="start").ok
    result = facade.start_go_to_station("no_such_place")
    assert result.error_code == ErrorCode.UNKNOWN_STATION


def test_hold_rejects_goto(facade: NavFacade):
    assert facade.init().ok
    assert facade.relocate(station_id="start").ok
    assert facade.hold("vision").ok
    assert facade.snapshot.nav_state == NavState.HOLDING
    result = facade.start_go_to_station("delivery_desk")
    assert result.error_code == ErrorCode.INVALID_STATE
    assert facade.release_hold().ok
    assert facade.snapshot.nav_state == NavState.READY


def test_cancel_nav(facade: NavFacade):
    events = []
    facade.on_event = events.append
    assert facade.init().ok
    assert facade.relocate(station_id="start").ok
    assert facade.start_go_to_station("shelf_group_a_front").ok
    assert facade.cancel_nav(reason="user").ok
    assert facade.snapshot.nav_state == NavState.READY
    assert any(e.type == EventType.CANCELLED.value for e in events)


def test_cannot_map_while_navigating(facade: NavFacade):
    assert facade.init().ok
    assert facade.relocate(station_id="start").ok
    assert facade.start_go_to_station("delivery_desk").ok
    result = facade.start_mapping("x")
    assert result.error_code == ErrorCode.INVALID_STATE
