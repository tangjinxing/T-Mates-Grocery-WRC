from __future__ import annotations

import copy
import math
import threading
from pathlib import Path
from typing import Any

import yaml


COMPONENTS = ("torso", "head", "left_arm", "right_arm")


class PoseRegistryError(RuntimeError):
    def __init__(self, error_code: str, message: str):
        super().__init__(message)
        self.error_code = error_code
        self.message = message


class PoseRegistry:
    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.RLock()

    def _load(self) -> dict[str, Any]:
        if not self.path.is_file():
            raise PoseRegistryError("CONFIG_NOT_FOUND", f"姿态配置不存在: {self.path}")
        try:
            data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise PoseRegistryError("CONFIG_INVALID", f"YAML解析失败: {exc}") from exc
        if not isinstance(data, dict) or not isinstance(data.get("poses"), dict):
            raise PoseRegistryError("CONFIG_INVALID", "YAML缺少poses映射")
        return data

    @staticmethod
    def _finite_pose(value: Any, field: str) -> list[float]:
        if not isinstance(value, list) or len(value) != 6:
            raise PoseRegistryError("POSE_INCOMPLETE", f"{field}必须包含6个数")
        result = [float(item) for item in value]
        if not all(math.isfinite(item) for item in result):
            raise PoseRegistryError("CONFIG_INVALID", f"{field}包含非法数值")
        return result

    def resolve(self, pose_id: str) -> dict[str, Any]:
        with self._lock:
            data = self._load()
            pose = data["poses"].get(pose_id)
            if not isinstance(pose, dict):
                raise PoseRegistryError("POSE_NOT_FOUND", f"未知姿态: {pose_id}")
            applied = pose.get("apply_components")
            if not isinstance(applied, list) or not applied:
                raise PoseRegistryError("CONFIG_INVALID", f"{pose_id}.apply_components无效")
            if len(set(applied)) != len(applied) or any(item not in COMPONENTS for item in applied):
                raise PoseRegistryError("CONFIG_INVALID", f"{pose_id}.apply_components包含未知或重复部件")

            missing: list[str] = []
            resolved: dict[str, Any] = {}
            if "torso" in applied:
                height = pose.get("torso", {}).get("height_mm")
                if height is None:
                    missing.append("torso.height_mm")
                else:
                    resolved["torso"] = {"height_mm": int(height)}
            if "head" in applied:
                head = pose.get("head", {})
                if head.get("yaw") is None or head.get("pitch") is None:
                    missing.append("head.yaw/pitch")
                else:
                    resolved["head"] = {"yaw": int(head["yaw"]), "pitch": int(head["pitch"])}
            for component in ("left_arm", "right_arm"):
                if component in applied:
                    arm = pose.get(component, {})
                    if arm.get("pose_6d") is None:
                        missing.append(f"{component}.pose_6d")
                    else:
                        resolved[component] = {
                            "frame": arm.get("frame"),
                            "pose_6d": self._finite_pose(arm["pose_6d"], f"{pose_id}.{component}.pose_6d"),
                        }
            if missing or pose.get("status") != "ready":
                detail = ", ".join(missing) if missing else f"status={pose.get('status')}"
                raise PoseRegistryError("POSE_INCOMPLETE", f"姿态{pose_id}尚未就绪: {detail}")
            return {
                "pose_id": pose_id,
                "description": pose.get("description", ""),
                "apply_components": list(applied),
                "targets": copy.deepcopy(resolved),
            }

    def summary(self) -> dict[str, str]:
        with self._lock:
            data = self._load()
            return {
                name: str(value.get("status", "unknown"))
                for name, value in data["poses"].items()
                if isinstance(value, dict)
            }
