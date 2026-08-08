from __future__ import annotations

import json
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any


class PoseStore:
    """Small, atomic JSON store for the three initial photo poses."""

    VALID_NAMES = {"left", "right", "head"}

    def __init__(self, root: Path):
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)

    def _path(self, name: str) -> Path:
        if name not in self.VALID_NAMES:
            raise ValueError(f"未知预设: {name}")
        return self.root / f"{name}_initial_photo_pose.json"

    def load(self, name: str) -> dict[str, Any] | None:
        path = self._path(name)
        if not path.exists():
            return None
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def list_all(self) -> dict[str, Any]:
        return {name: self.load(name) for name in sorted(self.VALID_NAMES)}

    def save(self, name: str, payload: dict[str, Any]) -> Path:
        path = self._path(name)
        now = datetime.now().astimezone().isoformat(timespec="seconds")
        data = {"schema_version": 1, "preset": name, "saved_at": now, **payload}

        if path.exists():
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(path, path.with_suffix(f".{stamp}.bak.json"))

        fd, temp_name = tempfile.mkstemp(prefix=f".{path.stem}.", suffix=".tmp", dir=self.root)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(data, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_name, path)
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)
        return path
