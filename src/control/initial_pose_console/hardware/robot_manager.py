from __future__ import annotations

import math
import sys
import threading
import time
from typing import Any


class RobotManager:
    """Connection, telemetry and deliberately conservative motion control."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        sys.path.insert(0, config["paths"]["arm_api"])
        sys.path.insert(0, config["paths"]["head_api"])
        from realman_arm_api_api2 import RealmanArmClient
        from servo_api import HeadControlSDK

        self._arm_type = RealmanArmClient
        self._head_type = HeadControlSDK
        self._arms: dict[str, Any] = {"left": None, "right": None}
        self._head = None
        self._state_lock = threading.RLock()
        self._connection_lock = threading.RLock()
        self._motion_lock = threading.Lock()
        self._head_io_lock = threading.Lock()
        self._states: dict[str, Any] = {
            "arms": {
                "left": {"connected": False, "state": None, "error": None},
                "right": {"connected": False, "state": None, "error": None},
            },
            "head": {"connected": False, "yaw": None, "pitch": None, "error": None},
            "lift": {"state": None, "motion_enabled": bool(config["lift"]["enable_motion"]), "error": None},
            "motion_busy": False,
        }
        self._stop_event = threading.Event()
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, name="robot-telemetry", daemon=True)
        self._telemetry_thread.start()

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            import copy
            return copy.deepcopy(self._states)

    def connect_arm(self, side: str) -> dict[str, Any]:
        if side not in self._arms:
            raise ValueError(f"未知机械臂: {side}")
        with self._connection_lock:
            if self._arms[side] is not None and self._arms[side].connected:
                return self.status()["arms"][side]
            item = self.config["arms"][side]
            client = self._arm_type(ip=item["ip"], model=item.get("model", side), auto_connect=False)
            client.connect()
            state = client.get_state()
            self._arms[side] = client
            with self._state_lock:
                self._states["arms"][side] = {"connected": True, "state": self._arm_state(state), "error": None}
            return self.status()["arms"][side]

    def disconnect_arm(self, side: str) -> dict[str, Any]:
        if side not in self._arms:
            raise ValueError(f"未知机械臂: {side}")
        with self._connection_lock:
            client = self._arms[side]
            self._arms[side] = None
            if client is not None:
                client.disconnect()
            with self._state_lock:
                self._states["arms"][side] = {"connected": False, "state": None, "error": None}
        return self.status()["arms"][side]

    def connect_head(self) -> dict[str, Any]:
        with self._connection_lock:
            if self._head is not None and self._head.is_online():
                return self.status()["head"]
            item = self.config["head"]
            head = self._head_type(port=item["port"], baudrate=int(item["baudrate"]))
            if not head.connect():
                raise RuntimeError(f"无法连接头部串口 {item['port']}")
            self._head = head
            self._poll_head()
            return self.status()["head"]

    def disconnect_head(self) -> dict[str, Any]:
        with self._connection_lock:
            head = self._head
            self._head = None
            if head is not None:
                with self._head_io_lock:
                    head.disconnect()
            with self._state_lock:
                self._states["head"] = {"connected": False, "yaw": None, "pitch": None, "error": None}
        return self.status()["head"]

    @staticmethod
    def _arm_state(state) -> dict[str, Any]:
        return {
            "joints_deg": list(state.joints),
            "pose_base_to_gripper_m_rad": state.pose.as_list(),
            "errors": state.err,
        }

    def get_actual_arm_pose(self, side: str) -> list[float]:
        client = self._arms.get(side)
        if client is None or not client.connected:
            raise RuntimeError(f"{side}臂尚未连接")
        state = client.get_state()
        with self._state_lock:
            self._states["arms"][side] = {"connected": True, "state": self._arm_state(state), "error": None}
        return state.pose.as_list()

    def get_actual_head(self) -> dict[str, int]:
        if self._head is None or not self._head.is_online():
            raise RuntimeError("头部尚未连接")
        self._poll_head()
        state = self.status()["head"]
        if state["yaw"] is None or state["pitch"] is None:
            raise RuntimeError("无法读取头部实际位置")
        return {"yaw": int(state["yaw"]), "pitch": int(state["pitch"])}

    def get_actual_lift_height(self) -> int:
        side = self.config["lift"]["controller_arm"]
        client = self._arms.get(side)
        if client is None or not client.connected:
            raise RuntimeError(f"躯干高度读取依赖{side}臂连接")
        state = client.get_lift_status()
        with self._state_lock:
            self._states["lift"]["state"] = {
                "height": state.height, "current": state.current,
                "err": state.err, "mode": state.mode,
            }
            self._states["lift"]["error"] = None
        return int(state.height)

    def _validate_pose(self, pose: list[float]) -> list[float]:
        if len(pose) != 6:
            raise ValueError("6D位姿必须包含 [x,y,z,rx,ry,rz]")
        values = [float(v) for v in pose]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("位姿包含非法数值")
        limits = self.config["motion"]["position_limits_m"]
        for index, axis in enumerate(("x", "y", "z")):
            low, high = limits[axis]
            if not float(low) <= values[index] <= float(high):
                raise ValueError(f"{axis}={values[index]} 超出安全配置范围 [{low}, {high}]")
        low, high = self.config["motion"]["euler_limits_rad"]
        if any(not float(low) <= value <= float(high) for value in values[3:]):
            raise ValueError(f"姿态角超出安全配置范围 [{low}, {high}] rad")
        return values

    def move_arm_pose(self, side: str, pose: list[float], speed: float, confirmed: bool) -> dict[str, Any]:
        if not confirmed:
            raise ValueError("运动指令缺少二次确认")
        client = self._arms.get(side)
        if client is None or not client.connected:
            raise RuntimeError(f"{side}臂尚未连接")
        target = self._validate_pose(pose)
        maximum = float(self.config["motion"]["max_speed"])
        speed = float(speed)
        if not 1 <= speed <= maximum:
            raise ValueError(f"速度必须在 1～{maximum}%")
        if not self._motion_lock.acquire(blocking=False):
            raise RuntimeError("已有运动任务正在执行")
        with self._state_lock:
            self._states["motion_busy"] = True
        try:
            client.movej_p(target, v=speed, r=0, trajectory_connect=0, block=True)
            actual = self.get_actual_arm_pose(side)
            return {"side": side, "target": target, "actual": actual}
        finally:
            with self._state_lock:
                self._states["motion_busy"] = False
            self._motion_lock.release()

    def move_head(self, yaw: int, pitch: int, confirmed: bool) -> dict[str, Any]:
        if not confirmed:
            raise ValueError("运动指令缺少二次确认")
        if self._head is None or not self._head.is_online():
            raise RuntimeError("头部尚未连接")
        yaw, pitch = int(yaw), int(pitch)
        yaw_min, yaw_max = self.config["head"]["yaw_limits"]
        pitch_min, pitch_max = self.config["head"]["pitch_limits"]
        if not int(yaw_min) <= yaw <= int(yaw_max):
            raise ValueError(f"yaw 必须在 {yaw_min}～{yaw_max}")
        if not int(pitch_min) <= pitch <= int(pitch_max):
            raise ValueError(f"pitch 必须在 {pitch_min}～{pitch_max}")
        if not self._motion_lock.acquire(blocking=False):
            raise RuntimeError("已有运动任务正在执行")
        with self._state_lock:
            self._states["motion_busy"] = True
        try:
            with self._head_io_lock:
                if not self._head.rotate(1, pitch) or not self._head.rotate(2, yaw):
                    raise RuntimeError("头部运动指令发送失败")
            tolerance = int(self.config["head"]["arrival_tolerance"])
            deadline = time.monotonic() + 8.0
            actual = None
            while time.monotonic() < deadline:
                actual = self.get_actual_head()
                if abs(actual["yaw"] - yaw) <= tolerance and abs(actual["pitch"] - pitch) <= tolerance:
                    break
                time.sleep(0.2)
            if actual is None or abs(actual["yaw"] - yaw) > tolerance or abs(actual["pitch"] - pitch) > tolerance:
                raise RuntimeError(f"头部未在容差 ±{tolerance} 内到达目标，实际={actual}")
            return {"target": {"yaw": yaw, "pitch": pitch}, "actual": actual}
        finally:
            with self._state_lock:
                self._states["motion_busy"] = False
            self._motion_lock.release()

    def move_lift(self, height: int, speed: int, confirmed: bool) -> dict[str, Any]:
        lift = self.config["lift"]
        if not bool(lift["enable_motion"]):
            raise RuntimeError("升降柱运动尚未启用：需先确认设备安全行程并更新配置")
        if not confirmed:
            raise ValueError("运动指令缺少二次确认")
        height = int(height)
        if not int(lift["min_height"]) <= height <= int(lift["max_height"]):
            raise ValueError(f"目标高度必须在 {lift['min_height']}～{lift['max_height']} mm")
        speed = int(speed)
        max_speed = int(lift.get("max_speed", 20))
        if not 1 <= speed <= max_speed:
            raise ValueError(f"升降柱速度必须在 1～{max_speed}%")
        side = lift["controller_arm"]
        client = self._arms.get(side)
        if client is None or not client.connected:
            raise RuntimeError(f"升降柱控制依赖{side}臂连接")
        current = self.get_actual_lift_height()
        if not int(lift["min_height"]) <= current <= int(lift["max_height"]):
            raise RuntimeError(
                f"升降柱当前反馈={current}，不在配置的 {lift['min_height']}～{lift['max_height']} mm 范围；"
                "请先确认控制器高度单位，已拒绝发送运动命令"
            )
        if not self._motion_lock.acquire(blocking=False):
            raise RuntimeError("已有运动任务正在执行")
        with self._state_lock:
            self._states["motion_busy"] = True
        try:
            client.control_lift("to", speed=speed, height=height, block=True)
            actual = self.get_actual_lift_height()
            tolerance = int(lift.get("arrival_tolerance", 10))
            if abs(actual - height) > tolerance:
                raise RuntimeError(f"升降柱未在容差 ±{tolerance} mm 内到达目标，实际={actual} mm")
            return {"target_height": height, "actual_height": actual, "speed": speed}
        finally:
            with self._state_lock:
                self._states["motion_busy"] = False
            self._motion_lock.release()

    def stop_all(self) -> dict[str, Any]:
        errors = []
        for side, client in self._arms.items():
            if client is not None and client.connected:
                try:
                    client.move_stop()
                except Exception as exc:
                    errors.append(f"{side}臂停止失败: {exc}")
        lift_side = self.config["lift"]["controller_arm"]
        lift_client = self._arms.get(lift_side)
        if lift_client is not None and lift_client.connected:
            try:
                lift_client.control_lift("stop")
            except Exception as exc:
                errors.append(f"升降柱停止失败: {exc}")
        return {"ok": not errors, "errors": errors}

    def _poll_head(self) -> None:
        head = self._head
        if head is None or not head.is_online():
            return
        with self._head_io_lock:
            positions = head.read_positions([1, 2], timeout=0.25)
        with self._state_lock:
            previous = self._states["head"]
            self._states["head"] = {
                "connected": True,
                "yaw": positions.get(2, previous.get("yaw")),
                "pitch": positions.get(1, previous.get("pitch")),
                "error": None if positions else "本轮未读到头部位置",
            }

    def _telemetry_loop(self) -> None:
        while not self._stop_event.wait(0.5):
            for side in ("left", "right"):
                client = self._arms.get(side)
                if client is None or not client.connected:
                    continue
                try:
                    state = client.get_state()
                    with self._state_lock:
                        self._states["arms"][side] = {"connected": True, "state": self._arm_state(state), "error": None}
                except Exception as exc:
                    with self._state_lock:
                        self._states["arms"][side]["error"] = str(exc)
            try:
                self._poll_head()
            except Exception as exc:
                with self._state_lock:
                    self._states["head"]["error"] = str(exc)
            lift_side = self.config["lift"]["controller_arm"]
            client = self._arms.get(lift_side)
            if client is not None and client.connected:
                try:
                    state = client.get_lift_status()
                    with self._state_lock:
                        self._states["lift"]["state"] = {
                            "height": state.height, "current": state.current,
                            "err": state.err, "mode": state.mode,
                        }
                        self._states["lift"]["error"] = None
                except Exception as exc:
                    with self._state_lock:
                        self._states["lift"]["error"] = str(exc)

    def close(self) -> None:
        self._stop_event.set()
        self.stop_all()
        self.disconnect_head()
        for side in ("left", "right"):
            self.disconnect_arm(side)
        self._telemetry_thread.join(timeout=2.0)
