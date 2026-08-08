"""Tests for HTTP navigation idempotency without ROS."""

from __future__ import annotations

import threading
import time

from retail_nav_bridge.http_gateway_core import (
    HttpResult,
    IdempotencyRegistry,
    NavigationCapabilities,
    evaluate_navigation_health,
)


READY_CAPABILITIES = NavigationCapabilities(
    response_ok=True,
    runtime_ready=True,
    navigation_ready=True,
    localization_ready=True,
    motion_ready=True,
)


def _health(**overrides):
    values = {
        "action_server_ready": True,
        "health_service_ready": True,
        "nav_state": "READY",
        "nav_status_age_sec": 0.1,
        "capabilities": READY_CAPABILITIES,
        "capabilities_age_sec": 0.1,
        "stale_after_sec": 3.0,
    }
    values.update(overrides)
    return evaluate_navigation_health(**values)


def test_navigation_health_requires_all_bottom_capabilities():
    assert _health() == "READY"

    for field in (
        "response_ok",
        "runtime_ready",
        "navigation_ready",
        "localization_ready",
        "motion_ready",
    ):
        values = READY_CAPABILITIES.__dict__.copy()
        values[field] = False
        assert (
            _health(capabilities=NavigationCapabilities(**values)) == "ERROR"
        )


def test_navigation_health_reports_startup_and_stale_states():
    assert _health(nav_state=None, capabilities=None) == "STARTING"
    assert _health(nav_state="LOCALIZING") == "STARTING"
    assert _health(nav_status_age_sec=3.1) == "ERROR"
    assert _health(capabilities_age_sec=3.1) == "ERROR"
    assert _health(health_service_ready=False) == "ERROR"


def test_same_key_executes_physical_action_once():
    registry = IdempotencyRegistry()
    calls = 0
    started = threading.Event()
    release = threading.Event()
    results = []

    def operation():
        nonlocal calls
        calls += 1
        started.set()
        release.wait(timeout=1.0)
        return HttpResult(200, {"status": "SUCCEEDED"})

    first = threading.Thread(
        target=lambda: results.append(
            registry.execute("key-1", "delivery_place", operation, 1.0)
        )
    )
    second = threading.Thread(
        target=lambda: results.append(
            registry.execute("key-1", "delivery_place", operation, 1.0)
        )
    )
    first.start()
    assert started.wait(timeout=1.0)
    second.start()
    time.sleep(0.02)
    release.set()
    first.join()
    second.join()

    assert calls == 1
    assert results == [
        HttpResult(200, {"status": "SUCCEEDED"}),
        HttpResult(200, {"status": "SUCCEEDED"}),
    ]


def test_same_key_with_different_target_is_rejected():
    registry = IdempotencyRegistry()
    success = registry.execute(
        "key-1",
        "delivery_place",
        lambda: HttpResult(200, {"status": "SUCCEEDED"}),
        1.0,
    )
    conflict = registry.execute(
        "key-1",
        "receipt_viewpoint",
        lambda: HttpResult(200, {"status": "SUCCEEDED"}),
        1.0,
    )

    assert success.status_code == 200
    assert conflict == HttpResult(
        409, {"error_code": "IDEMPOTENCY_KEY_CONFLICT"}
    )


def test_terminal_failure_is_cached():
    registry = IdempotencyRegistry()
    calls = 0

    def operation():
        nonlocal calls
        calls += 1
        return HttpResult(500, {"error_code": "EXECUTION_FAILED"})

    first = registry.execute("key-1", "delivery_place", operation, 1.0)
    retry = registry.execute("key-1", "delivery_place", operation, 1.0)

    assert calls == 1
    assert first == retry
