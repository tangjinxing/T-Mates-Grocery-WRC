from __future__ import annotations

import copy
import sys
import threading
import time
from pathlib import Path
from typing import Any

import yaml

from services.motion_coordinator import MotionCoordinator


ARM_API_ROOT = Path("/home/lh/robot_api/arm_api_new")


class GripperInitializer:
    """Initialize both native grippers once during the API lifespan startup."""

    def __init__(self, config_path: Path, coordinator: MotionCoordinator) -> None:
        self.config_path = config_path
        self.coordinator = coordinator
        self._lock = threading.RLock()
        self._status: dict[str, Any] = {
            "enabled": None,
            "completed": False,
            "arms": {
                "left": {"initialized": False, "message": "尚未执行启动初始化"},
                "right": {"initialized": False, "message": "尚未执行启动初始化"},
            },
        }

    def _load_config(self) -> dict[str, Any]:
        if not self.config_path.is_file():
            raise RuntimeError(f"夹爪初始化配置不存在: {self.config_path}")
        data = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        config = data.get("gripper_startup")
        if not isinstance(config, dict):
            raise RuntimeError("hardware_init.yaml缺少gripper_startup")
        arms = config.get("arms")
        if not isinstance(arms, dict) or not all(side in arms for side in ("left", "right")):
            raise RuntimeError("夹爪初始化配置必须包含left和right")
        return config

    def initialize_all(self) -> dict[str, Any]:
        try:
            config = self._load_config()
        except Exception as exc:
            with self._lock:
                self._status["enabled"] = None
                self._status["completed"] = True
                self._status["configuration_error"] = str(exc)
            return self.snapshot()

        enabled = bool(config.get("enabled", True))
        with self._lock:
            self._status["enabled"] = enabled
        if not enabled:
            with self._lock:
                self._status["completed"] = True
                for side in ("left", "right"):
                    self._status["arms"][side] = {
                        "initialized": False,
                        "message": "启动夹爪初始化已被配置禁用",
                    }
            return self.snapshot()

        self.coordinator.acquire()
        try:
            for side in ("left", "right"):
                result = self._initialize_one(side, config)
                with self._lock:
                    self._status["arms"][side] = result
            with self._lock:
                self._status["completed"] = True
        finally:
            self.coordinator.release()
        return self.snapshot()

    def _initialize_one(self, side: str, config: dict[str, Any]) -> dict[str, Any]:
        sys.path.insert(0, str(ARM_API_ROOT))
        from realman_arm_api_api2 import RealmanArmClient

        arm_config = config["arms"][side]
        client = RealmanArmClient(
            ip=str(arm_config["ip"]),
            model=str(arm_config.get("model", side)),
            auto_connect=False,
        )
        try:
            client.connect()
            if not client.get_power_state():
                raise RuntimeError("机械臂未上电")

            expected_voltage = int(config.get("voltage_type", 3))
            voltage = int(client.get_tool_voltage())
            if voltage != expected_voltage:
                if not bool(config.get("allow_set_24v", False)):
                    raise RuntimeError(
                        f"工具端电压类型为{voltage}，期望{expected_voltage}且禁止自动修改"
                    )
                client.set_tool_voltage(expected_voltage)
                time.sleep(2.0)
                voltage = int(client.get_tool_voltage())
                if voltage != expected_voltage:
                    raise RuntimeError(
                        f"工具端电压设置后仍为{voltage}，期望{expected_voltage}"
                    )

            client.configure_gripper_range(
                int(config.get("route_min", 0)),
                int(config.get("route_max", 1000)),
            )
            if bool(config.get("open_on_startup", True)):
                client.gripper_release(
                    speed=int(config.get("open_speed", 100)),
                    block=True,
                    timeout=int(config.get("command_timeout_s", 10)),
                )

            deadline = time.monotonic() + float(config.get("verify_timeout_s", 8.0))
            minimum = int(config.get("minimum_open_position", 990))
            last_state = None
            while time.monotonic() < deadline:
                last_state = client.get_gripper_state()
                online = (
                    int(last_state.enable_state) == 1
                    and int(last_state.status) == 1
                    and int(last_state.error) == 0
                    and int(last_state.temperature) > 0
                )
                opened = not bool(config.get("open_on_startup", True)) or int(last_state.actpos) >= minimum
                if online and opened:
                    return {
                        "initialized": True,
                        "message": "夹爪已初始化并在线",
                        "ip": str(arm_config["ip"]),
                        "voltage_type": voltage,
                        "enable_state": int(last_state.enable_state),
                        "status": int(last_state.status),
                        "error": int(last_state.error),
                        "mode": int(last_state.mode),
                        "temperature": int(last_state.temperature),
                        "actpos": int(last_state.actpos),
                    }
                time.sleep(float(config.get("verify_poll_s", 0.5)))
            raise RuntimeError(f"夹爪状态验证超时，最后状态={last_state}")
        except Exception as exc:
            return {
                "initialized": False,
                "message": str(exc),
                "ip": str(arm_config.get("ip", "")),
            }
        finally:
            try:
                client.disconnect()
            except Exception:
                pass

    def is_ready(self, hand: str) -> bool:
        side = hand.lower()
        with self._lock:
            return bool(self._status.get("arms", {}).get(side, {}).get("initialized"))

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._status)
