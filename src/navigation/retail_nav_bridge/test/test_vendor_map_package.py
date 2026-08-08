"""Tests for native TX-S2 map-package export (no ROS required)."""

from __future__ import annotations

from pathlib import Path

import yaml

from retail_nav_bridge.adapters.mock import MockNavAdapter
from retail_nav_bridge.constants import ErrorCode
from retail_nav_bridge.facade import NavFacade
from retail_nav_bridge.map_package import export_vendor_map_package
from retail_nav_bridge.stations import load_station_catalog

STATIONS = Path(__file__).resolve().parents[1] / "config" / "retail_stations.yaml"


def test_vendor_map_export_writes_native_files_only(tmp_path: Path):
    package_dir, statuses = export_vendor_map_package(
        tmp_path,
        {0: b"bmap-data", 1: b"png-data", 3: b"pcd-data"},
        map_id="7",
        map_name="HMI map / A",
        notes="edited in HMI",
    )

    assert package_dir.parent == tmp_path
    assert (package_dir / "map.bmap").read_bytes() == b"bmap-data"
    assert (package_dir / "map.png").read_bytes() == b"png-data"
    assert not (package_dir / "map.xml").exists()
    assert (package_dir / "map.pcd").read_bytes() == b"pcd-data"
    assert not (package_dir / "stations.yaml").exists()
    assert statuses == {0: "exported", 1: "exported", 2: "unavailable", 3: "exported"}

    manifest = yaml.safe_load((package_dir / "manifest.yaml").read_text(encoding="utf-8"))
    assert manifest["artifact_type"] == "tx_s2_vendor_map"
    assert manifest["map_id"] == "7"
    assert manifest["formats"]["bmap"] == "exported"


def test_vendor_map_export_requires_bmap(tmp_path: Path):
    try:
        export_vendor_map_package(tmp_path, {1: b"png"})
    except ValueError as exc:
        assert "bmap" in str(exc)
    else:
        raise AssertionError("bmap must be required")


def test_facade_vendor_export_is_not_available_for_mock(tmp_path: Path):
    facade = NavFacade(
        MockNavAdapter(),
        load_station_catalog(STATIONS),
        stations_file=str(tmp_path / "retail_stations.yaml"),
    )
    result = facade.export_vendor_map_package(str(tmp_path))
    assert result.error_code == ErrorCode.NOT_READY
