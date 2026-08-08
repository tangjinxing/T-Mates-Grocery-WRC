"""HTTP/JSON gateway for the retail navigation ROS2 action."""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from retail_nav_msgs.action import GoToStation
from retail_nav_msgs.msg import NavStatus
from retail_nav_msgs.srv import GetHealth

from retail_nav_bridge.http_gateway_core import (
    HttpResult,
    IdempotencyRegistry,
    NavigationCapabilities,
    evaluate_navigation_health,
)
from retail_nav_bridge.target_resolver import (
    TargetResolutionError,
    load_target_resolver,
)


def _default_config_path(filename: str) -> str:
    source_candidate = Path(__file__).resolve().parents[1] / "config" / filename
    if source_candidate.exists():
        return str(source_candidate)
    return str(
        Path(get_package_share_directory("retail_nav_bridge"))
        / "config"
        / filename
    )


class _GatewayHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, server_address, handler_class, gateway) -> None:
        super().__init__(server_address, handler_class)
        self.gateway = gateway


class _GatewayRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "TianJiNavigationGateway/1.0"
    max_body_bytes = 16 * 1024

    @property
    def gateway(self):
        return self.server.gateway  # type: ignore[attr-defined]

    def _write_json(self, result: HttpResult) -> None:
        payload = json.dumps(
            result.body, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.send_response(result.status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        if urlsplit(self.path).path != "/navigation/health":
            self._write_json(HttpResult(404, {"error_code": "NOT_FOUND"}))
            return
        self._write_json(HttpResult(200, {"status": self.gateway.health_status()}))

    def do_POST(self) -> None:
        if urlsplit(self.path).path != "/navigation/navigate":
            self._write_json(HttpResult(404, {"error_code": "NOT_FOUND"}))
            return

        idempotency_key = self.headers.get("Idempotency-Key", "").strip()
        if not idempotency_key:
            self._write_json(
                HttpResult(400, {"error_code": "INVALID_REQUEST"})
            )
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            content_length = -1
        if content_length <= 0 or content_length > self.max_body_bytes:
            self._write_json(
                HttpResult(400, {"error_code": "INVALID_REQUEST"})
            )
            return

        try:
            body = json.loads(self.rfile.read(content_length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._write_json(
                HttpResult(400, {"error_code": "INVALID_REQUEST"})
            )
            return

        if (
            not isinstance(body, dict)
            or set(body) != {"target_id"}
            or not isinstance(body["target_id"], str)
            or not body["target_id"].strip()
        ):
            self._write_json(
                HttpResult(400, {"error_code": "INVALID_REQUEST"})
            )
            return

        target_id = body["target_id"].strip()
        result = self.gateway.execute_idempotent(
            idempotency_key=idempotency_key,
            target_id=target_id,
        )
        self._write_json(result)

    def log_message(self, format_string: str, *args) -> None:
        self.gateway.get_logger().debug(format_string % args)


class NavigationHttpGateway(Node):
    def __init__(self) -> None:
        super().__init__("retail_nav_http_gateway")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8081)
        self.declare_parameter("navigation_timeout_sec", 120.0)
        self.declare_parameter("action_server_timeout_sec", 5.0)
        self.declare_parameter("health_status_stale_sec", 3.0)
        self.declare_parameter("health_poll_sec", 1.0)
        self.declare_parameter("default_max_speed", 0.3)
        self.declare_parameter(
            "stations_file", _default_config_path("retail_stations.yaml")
        )
        self.declare_parameter(
            "target_mapping_file",
            _default_config_path("product_slot_navigation.yaml"),
        )

        self._navigation_timeout_sec = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        self._action_server_timeout_sec = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self._health_status_stale_sec = float(
            self.get_parameter("health_status_stale_sec").value
        )
        self._health_poll_sec = float(
            self.get_parameter("health_poll_sec").value
        )
        self._default_max_speed = float(
            self.get_parameter("default_max_speed").value
        )
        stations_file = str(self.get_parameter("stations_file").value)
        target_mapping_file = str(
            self.get_parameter("target_mapping_file").value
        )
        self._target_resolver = load_target_resolver(
            target_mapping_file,
            stations_file,
        )

        self._status_lock = threading.Lock()
        self._nav_state: Optional[str] = None
        self._status_received_at = 0.0
        self._capabilities: Optional[NavigationCapabilities] = None
        self._capabilities_received_at = 0.0
        self._health_request_pending = False
        self._registry = IdempotencyRegistry()

        self._action_client = ActionClient(
            self,
            GoToStation,
            "/retail_nav/v1/GoToStation",
        )
        self.create_subscription(
            NavStatus,
            "/retail_nav/v1/status",
            self._on_status,
            10,
        )
        self._health_client = self.create_client(
            GetHealth,
            "/retail_nav/v1/GetHealth",
        )
        self.create_timer(self._health_poll_sec, self._poll_ros_health)

        host = str(self.get_parameter("http_host").value)
        port = int(self.get_parameter("http_port").value)
        self._http_server = _GatewayHttpServer(
            (host, port),
            _GatewayRequestHandler,
            self,
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="retail-nav-http",
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(
            f"navigation HTTP gateway listening on {host}:{port}"
        )

    def _on_status(self, message: NavStatus) -> None:
        with self._status_lock:
            self._nav_state = message.nav_state
            self._status_received_at = time.monotonic()

    def _poll_ros_health(self) -> None:
        with self._status_lock:
            if self._health_request_pending:
                return
        if not self._health_client.service_is_ready():
            return

        with self._status_lock:
            self._health_request_pending = True
        try:
            future = self._health_client.call_async(GetHealth.Request())
            future.add_done_callback(self._on_ros_health)
        except Exception as exc:
            self.get_logger().error(f"failed to call GetHealth: {exc}")
            with self._status_lock:
                self._health_request_pending = False

    def _on_ros_health(self, future) -> None:
        try:
            response = future.result()
            health = response.health
            capabilities = NavigationCapabilities(
                response_ok=response.error_code == 0,
                runtime_ready=health.runtime_ready,
                navigation_ready=health.navigation_ready,
                localization_ready=health.localization_ready,
                motion_ready=health.motion_ready,
            )
            received_at = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"GetHealth request failed: {exc}")
            capabilities = NavigationCapabilities(
                response_ok=False,
                runtime_ready=False,
                navigation_ready=False,
                localization_ready=False,
                motion_ready=False,
            )
            received_at = time.monotonic()

        with self._status_lock:
            self._capabilities = capabilities
            self._capabilities_received_at = received_at
            self._health_request_pending = False

    def health_status(self) -> str:
        now = time.monotonic()
        with self._status_lock:
            nav_state = self._nav_state
            status_received_at = self._status_received_at
            capabilities = self._capabilities
            capabilities_received_at = self._capabilities_received_at

        return evaluate_navigation_health(
            action_server_ready=self._action_client.server_is_ready(),
            health_service_ready=self._health_client.service_is_ready(),
            nav_state=nav_state,
            nav_status_age_sec=(
                now - status_received_at if status_received_at else 0.0
            ),
            capabilities=capabilities,
            capabilities_age_sec=(
                now - capabilities_received_at
                if capabilities_received_at
                else 0.0
            ),
            stale_after_sec=self._health_status_stale_sec,
        )

    def execute_idempotent(
        self, idempotency_key: str, target_id: str
    ) -> HttpResult:
        wait_timeout = (
            self._navigation_timeout_sec
            + self._action_server_timeout_sec
            + 15.0
        )
        return self._registry.execute(
            key=idempotency_key,
            target_id=target_id,
            operation=lambda: self._resolve_and_navigate(
                target_id, idempotency_key
            ),
            wait_timeout_sec=wait_timeout,
        )

    def _resolve_and_navigate(
        self, target_id: str, request_id: str
    ) -> HttpResult:
        try:
            station_id = self._target_resolver.resolve(target_id)
        except TargetResolutionError as exc:
            self.get_logger().error(str(exc))
            return HttpResult(400, {"error_code": "INVALID_TARGET"})
        self.get_logger().info(
            f"resolved navigation target {target_id} -> {station_id}"
        )
        return self._navigate(station_id, request_id)

    def _navigate(self, target_id: str, request_id: str) -> HttpResult:
        if self.health_status() != "READY":
            return HttpResult(503, {"error_code": "EXECUTION_FAILED"})
        if not self._action_client.wait_for_server(
            timeout_sec=self._action_server_timeout_sec
        ):
            return HttpResult(503, {"error_code": "EXECUTION_FAILED"})

        completed = threading.Event()
        result_holder: list[HttpResult] = []
        goal_handle_holder = []

        def finish(result: HttpResult) -> None:
            if not completed.is_set():
                result_holder.append(result)
                completed.set()

        def on_result(future) -> None:
            try:
                action_result = future.result().result
                if (
                    action_result.error_code == 0
                    and action_result.result_code == "arrived"
                ):
                    finish(HttpResult(200, {"status": "SUCCEEDED"}))
                    return
                self.get_logger().error(
                    "navigation failed: target=%s code=%s result=%s message=%s"
                    % (
                        target_id,
                        action_result.error_code,
                        action_result.result_code,
                        action_result.error_msg,
                    )
                )
            except Exception as exc:
                self.get_logger().error(
                    f"failed to receive navigation result for {target_id}: {exc}"
                )
            finish(HttpResult(500, {"error_code": "EXECUTION_FAILED"}))

        def on_goal_response(future) -> None:
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().error(
                        f"navigation goal rejected: target={target_id}"
                    )
                    finish(HttpResult(500, {"error_code": "EXECUTION_FAILED"}))
                    return
                goal_handle_holder.append(goal_handle)
                goal_handle.get_result_async().add_done_callback(on_result)
            except Exception as exc:
                self.get_logger().error(
                    f"failed to send navigation goal for {target_id}: {exc}"
                )
                finish(HttpResult(500, {"error_code": "EXECUTION_FAILED"}))

        goal = GoToStation.Goal()
        goal.station_id = target_id
        goal.request_id = request_id
        goal.max_speed = self._default_max_speed
        goal.timeout_sec = self._navigation_timeout_sec
        try:
            self._action_client.send_goal_async(goal).add_done_callback(
                on_goal_response
            )
        except Exception as exc:
            self.get_logger().error(
                f"failed to create navigation goal for {target_id}: {exc}"
            )
            return HttpResult(500, {"error_code": "EXECUTION_FAILED"})

        wait_timeout = self._navigation_timeout_sec + 15.0
        if not completed.wait(timeout=wait_timeout):
            self.get_logger().error(
                f"navigation result timed out: target={target_id}"
            )
            if goal_handle_holder:
                goal_handle_holder[0].cancel_goal_async()
            return HttpResult(504, {"error_code": "EXECUTION_FAILED"})
        return result_holder[0]

    def destroy_node(self) -> bool:
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_thread.join(timeout=2.0)
        self._action_client.destroy()
        self.destroy_client(self._health_client)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationHttpGateway()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
