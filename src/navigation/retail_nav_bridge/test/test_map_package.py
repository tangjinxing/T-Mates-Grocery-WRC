"""Tests for map package export / SyncStations (no ROS)."""

from __future__ import annotations

from pathlib import Path

from retail_nav_bridge.adapters.mock import MockNavAdapter
from retail_nav_bridge.constants import ErrorCode, NavState
from retail_nav_bridge.facade import NavFacade
from retail_nav_bridge.map_package import (
    export_map_package,
    load_package_stations,
    sync_stations_to_file,
)
from retail_nav_bridge.stations import load_station_catalog

STATIONS = Path(__file__).resolve().parents[1] / "config" / "retail_stations.yaml"


def test_export_and_sync_roundtrip(tmp_path: Path):
    catalog = load_station_catalog(STATIONS)
    pkg = export_map_package(
        tmp_path / "pkg",
        catalog,
        map_id="3",
        map_name="retail_default",
        map_path="/vendor/maps/retail.yaml",
        notes="unit-test",
    )
    assert (pkg / "manifest.yaml").is_file()
    assert (pkg / "stations.yaml").is_file()

    loaded = load_package_stations(pkg)
    assert loaded.get("delivery_desk") is not None
    assert abs(loaded.get("delivery_desk").x - catalog.get("delivery_desk").x) < 1e-9

    target = tmp_path / "retail_stations.yaml"
    synced = sync_stations_to_file(pkg, target)
    assert target.is_file()
    assert synced.get("start") is not None
    reloaded = load_station_catalog(target)
    assert "judge_zone_1" in reloaded.stations


def test_facade_sync_stations_hot_reload(tmp_path: Path):
    catalog = load_station_catalog(STATIONS)
    pkg = export_map_package(tmp_path / "pkg", catalog, map_name="retail_default")
    stations_yaml = pkg / "stations.yaml"

    from retail_nav_bridge.stations import Station, StationCatalog, save_station_catalog

    stations = dict(catalog.stations)
    old = stations["delivery_desk"]
    stations["delivery_desk"] = Station(
        station_id=old.station_id,
        display_name=old.display_name,
        x=1.111,
        y=old.y,
        yaw=old.yaw,
        arrive_tol_m=old.arrive_tol_m,
        yaw_tol_rad=old.yaw_tol_rad,
        max_speed=old.max_speed,
        vendor_target_id=old.vendor_target_id,
    )
    save_station_catalog(
        stations_yaml, StationCatalog(map_name="retail_default", stations=stations)
    )

    target = tmp_path / "active_stations.yaml"
    facade = NavFacade(
        MockNavAdapter(),
        catalog,
        stations_file=str(target),
    )
    assert facade.init().ok
    result = facade.sync_stations(str(pkg))
    assert result.ok
    assert result.station_count >= 1
    assert facade.catalog.get("delivery_desk").x == 1.111
    assert load_station_catalog(target).get("delivery_desk").x == 1.111


def test_sync_rejected_while_navigating(tmp_path: Path):
    catalog = load_station_catalog(STATIONS)
    pkg = export_map_package(tmp_path / "pkg", catalog)
    target = tmp_path / "active.yaml"
    facade = NavFacade(MockNavAdapter(linear_speed=2.0), catalog, stations_file=str(target))
    assert facade.init().ok
    assert facade.relocate(station_id="start").ok
    assert facade.start_go_to_station("delivery_desk").ok
    assert facade.snapshot.nav_state == NavState.NAVIGATING
    result = facade.sync_stations(str(pkg))
    assert result.error_code == ErrorCode.INVALID_STATE


def test_load_map_does_not_change_catalog(tmp_path: Path):
    catalog = load_station_catalog(STATIONS)
    x_before = catalog.get("delivery_desk").x
    facade = NavFacade(MockNavAdapter(), catalog, stations_file=str(tmp_path / "s.yaml"))
    assert facade.init().ok
    assert facade.load_map(map_name="retail_default").ok
    assert facade.catalog.get("delivery_desk").x == x_before
