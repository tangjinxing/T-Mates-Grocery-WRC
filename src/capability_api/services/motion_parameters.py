from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class MotionParameterError(RuntimeError):
    pass


class MotionParameters:
    """Load validated operator-tunable motion parameters once at service startup."""

    _LIMITS = {
        "pose_prepare.arm_movej_p_speed_percent": (1.0, 40.0),
        "pose_prepare.lift_speed": (1.0, 40.0),
        "grasp.pregrasp_movej_p_speed_percent": (1.0, 40.0),
        "grasp.approach_movel_speed_percent": (1.0, 40.0),
        "grasp.return_movel_speed_percent": (1.0, 40.0),
        "grasp.return_movej_p_speed_percent": (1.0, 40.0),
        "grasp.gripper_speed": (1.0, 1000.0),
        "grasp.gripper_force": (50.0, 500.0),
        "release.movej_p_speed_percent": (1.0, 40.0),
    }

    def __init__(self, path: Path) -> None:
        self.path = path
        self._values = self._load()

    def _load(self) -> dict[str, Any]:
        if not self.path.is_file():
            raise MotionParameterError(f"运动参数配置不存在: {self.path}")
        try:
            data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise MotionParameterError(f"运动参数YAML解析失败: {exc}") from exc
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise MotionParameterError("运动参数配置schema_version必须为1")
        for dotted_name, (minimum, maximum) in self._LIMITS.items():
            section, name = dotted_name.split(".", 1)
            raw = data.get(section, {}).get(name) if isinstance(data.get(section), dict) else None
            if not isinstance(raw, (int, float)) or isinstance(raw, bool):
                raise MotionParameterError(f"运动参数{dotted_name}必须为数值")
            value = float(raw)
            if not minimum <= value <= maximum:
                raise MotionParameterError(
                    f"运动参数{dotted_name}={value:g}超出代码安全范围{minimum:g}～{maximum:g}"
                )
        return data

    def get(self, section: str, name: str) -> float:
        return float(self._values[section][name])

    def summary(self) -> dict[str, Any]:
        return {key: dict(value) for key, value in self._values.items() if isinstance(value, dict)}
