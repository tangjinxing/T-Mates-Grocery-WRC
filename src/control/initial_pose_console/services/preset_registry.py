from __future__ import annotations

import os
import re
import shutil
import tempfile
import threading
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


class PresetRegistry:
    NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]{2,63}$")
    COMPONENTS = {"head", "torso", "left_arm", "right_arm"}

    def __init__(self, path: Path, backup_root: Path):
        self.path = path
        self.backup_root = backup_root
        self._lock = threading.RLock()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.backup_root.mkdir(parents=True, exist_ok=True)

    def _load(self) -> dict[str, Any]:
        if not self.path.is_file():
            raise FileNotFoundError(f"预设文件不存在: {self.path}")
        data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("presets"), dict):
            raise ValueError("preset_poses.yaml 结构无效")
        return data

    def all(self) -> dict[str, Any]:
        with self._lock:
            return self._load()

    def get(self, name: str) -> dict[str, Any]:
        with self._lock:
            data = self._load()
            if name not in data["presets"]:
                raise ValueError(f"预设不存在: {name}")
            return data["presets"][name]

    def _validate_name(self, name: str) -> None:
        if not self.NAME_PATTERN.fullmatch(name):
            raise ValueError("名称必须以小写字母开头，只含小写字母、数字和下划线，长度3～64")

    @staticmethod
    def _empty_preset(description: str) -> dict[str, Any]:
        return {
            "description": description,
            "apply_components": [],
            "status": "pending",
            "updated_at": None,
            "source": None,
            "head": {"yaw": None, "pitch": None},
            "torso": {"height": None, "unit": "mm"},
            "left_arm": {"frame": "left_arm_base", "pose_6d": None},
            "right_arm": {"frame": "right_arm_base", "pose_6d": None},
        }

    def create(self, name: str, description: str) -> dict[str, Any]:
        self._validate_name(name)
        description = description.strip()
        if not description:
            raise ValueError("预设说明不能为空")
        with self._lock:
            data = self._load()
            if name in data["presets"]:
                raise ValueError(f"预设已存在: {name}")
            data["presets"][name] = self._empty_preset(description)
            self._write(data)
            return data["presets"][name]

    @staticmethod
    def _status(preset: dict[str, Any]) -> str:
        applied = preset.get("apply_components", [])
        if not applied:
            return "pending"
        ready = {
            "head": preset.get("head", {}).get("yaw") is not None
                    and preset.get("head", {}).get("pitch") is not None,
            "torso": preset.get("torso", {}).get("height") is not None,
            "left_arm": preset.get("left_arm", {}).get("pose_6d") is not None,
            "right_arm": preset.get("right_arm", {}).get("pose_6d") is not None,
        }
        available_count = sum(bool(ready.get(component)) for component in applied)
        if available_count == len(applied):
            return "ready"
        return "pending" if available_count == 0 else "incomplete"

    def update_apply_components(self, name: str, components: list[str]) -> dict[str, Any]:
        ordered = []
        for component in components:
            if component not in self.COMPONENTS:
                raise ValueError(f"未知部件: {component}")
            if component not in ordered:
                ordered.append(component)
        if not ordered:
            raise ValueError("至少选择一个参与执行的部件")
        with self._lock:
            data = self._load()
            if name not in data["presets"]:
                raise ValueError(f"预设不存在: {name}")
            preset = data["presets"][name]
            preset["apply_components"] = ordered
            preset["status"] = self._status(preset)
            preset["updated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            preset["source"] = "preset_manager"
            self._write(data)
            return preset

    def update_component(self, name: str, component: str, value: Any) -> dict[str, Any]:
        if component not in self.COMPONENTS:
            raise ValueError(f"未知部件: {component}")
        with self._lock:
            data = self._load()
            if name not in data["presets"]:
                raise ValueError(f"预设不存在: {name}")
            preset = data["presets"][name]
            if component == "head":
                preset["head"] = {"yaw": int(value["yaw"]), "pitch": int(value["pitch"])}
            elif component == "torso":
                height = int(value)
                if not 100 <= height <= 1350:
                    raise ValueError(f"躯干实际高度 {height} mm 超出安全范围 100～1350 mm，拒绝保存")
                preset["torso"] = {"height": height, "unit": "mm"}
            else:
                pose = [float(item) for item in value]
                if len(pose) != 6:
                    raise ValueError("机械臂实际位姿长度必须为6")
                preset[component] = {
                    "frame": "left_arm_base" if component == "left_arm" else "right_arm_base",
                    "pose_6d": pose,
                }
            preset["status"] = self._status(preset)
            preset["updated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            preset["source"] = "actual_robot_state"
            self._write(data)
            return preset

    def _write(self, data: dict[str, Any]) -> None:
        if self.path.exists():
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            shutil.copy2(self.path, self.backup_root / f"preset_poses_{stamp}.yaml")
        fd, temp_name = tempfile.mkstemp(prefix=".preset_poses_", suffix=".yaml.tmp", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False, width=120)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_name, self.path)
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)
