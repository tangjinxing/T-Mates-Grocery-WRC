from __future__ import annotations

import threading


class MotionCoordinator:
    """Serialize all robot actions and hardware-backed plans in this process."""

    def __init__(self) -> None:
        self._lock = threading.Lock()

    def acquire(self) -> None:
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("已有机器人动作或抓取规划正在执行")

    def release(self) -> None:
        self._lock.release()
