"""Tests for Agent target_id to navigation station resolution."""

from pathlib import Path

import pytest

from retail_nav_bridge.target_resolver import (
    TargetResolutionError,
    load_target_resolver,
)

FIXTURES = Path(__file__).resolve().parent / "fixtures"


@pytest.fixture
def resolver():
    # Use full 20-station fixture; live config/retail_stations.yaml may be a
    # reduced A→B smoke subset while product_slot_navigation.yaml stays complete.
    return load_target_resolver(
        FIXTURES / "product_slot_navigation.yaml",
        FIXTURES / "retail_stations_full.yaml",
    )


@pytest.mark.parametrize(
    ("target_id", "expected"),
    [
        ("H1_F_L1_C01", "shelf_group_a_front"),
        ("H1_F_L1_C03", "shelf_group_a_front_2"),
        ("H1_F_L1_C05", "shelf_group_a_front_3"),
        ("H1_F_L1_C07", "shelf_group_a_front_4"),
        ("H1_F_L2_C04", "shelf_group_a_front_2"),
        ("H1_F_L2_C13", "shelf_group_a_front_4"),
        ("H1_B_L3_C06", "shelf_group_a_back_4"),
        ("H2_F_L4_C08", "shelf_group_b_front_4"),
        ("H2_B_L4_C03", "shelf_group_b_back_4"),
        ("H2_B_L5_C01", "shelf_group_b_back"),
        ("H2_B_L5_C02", "shelf_group_b_back_3"),
    ],
)
def test_each_level_uses_its_own_column_count(
    resolver, target_id, expected
):
    assert resolver.resolve(target_id) == expected


def test_same_column_can_map_differently_on_different_levels(resolver):
    assert resolver.resolve("H1_F_L1_C04") == "shelf_group_a_front_2"
    assert resolver.resolve("H1_F_L2_C04") == "shelf_group_a_front_2"
    assert resolver.resolve("H1_F_L4_C04") == "shelf_group_a_front_3"


def test_all_columns_from_current_placement_table_are_resolvable(resolver):
    level_counts = {
        "H1_F": [7, 13, 7, 5, 5],
        "H1_B": [7, 7, 6, 6, 6],
        "H2_F": [6, 5, 6, 8, 7],
        "H2_B": [5, 5, 6, 3, 2],
    }

    for face_id, counts in level_counts.items():
        for level, column_count in enumerate(counts, start=1):
            for column in range(1, column_count + 1):
                station_id = resolver.resolve(
                    f"{face_id}_L{level}_C{column:02d}"
                )
                assert station_id.startswith("shelf_group_")
            with pytest.raises(TargetResolutionError):
                resolver.resolve(
                    f"{face_id}_L{level}_C{column_count + 1:02d}"
                )


def test_business_aliases_and_direct_station_ids(resolver):
    assert resolver.resolve("delivery_place") == "delivery_desk"
    assert resolver.resolve("replenishment_pickup") == "restock_desk"
    assert resolver.resolve("task_boundary") == "judge_zone_1"
    assert resolver.resolve("delivery_desk") == "delivery_desk"


@pytest.mark.parametrize(
    "target_id",
    [
        "H1_F_L1_C00",
        "H1_F_L1_C08",
        "H1_F_L2_C14",
        "H2_B_L5_C03",
        "H3_F_L1_C01",
        "unknown",
    ],
)
def test_invalid_or_unmapped_target_is_rejected(resolver, target_id):
    with pytest.raises(TargetResolutionError):
        resolver.resolve(target_id)


def test_hotel1_c01_c03_left_c04_right():
    config = Path(__file__).resolve().parents[1] / "config"
    hotel = load_target_resolver(
        config / "product_slot_navigation.yaml",
        config / "retail_stations.yaml",
    )
    assert hotel.resolve("H1_F_L1_C01") == "mark_5"
    assert hotel.resolve("H1_F_L1_C03") == "mark_5"
    assert hotel.resolve("H1_F_L1_C04") == "mark_4"
    assert hotel.resolve("H1_F_L1_C06") == "mark_4"
    assert hotel.resolve("H1_B_L3_C03") == "mark_2"
    assert hotel.resolve("H1_B_L3_C04") == "mark_3"
    assert hotel.resolve("H1_B_L3_C06") == "mark_3"
