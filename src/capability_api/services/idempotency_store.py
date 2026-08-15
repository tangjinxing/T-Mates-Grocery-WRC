from __future__ import annotations

import json
import sqlite3
import threading
from datetime import datetime
from pathlib import Path
from typing import Any


class IdempotencyStore:
    """Persistent physical-action results. RUNNING is never auto-retried after restart."""

    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        with self._connect() as db:
            db.execute(
                """
                CREATE TABLE IF NOT EXISTS actions (
                    key TEXT PRIMARY KEY,
                    request_hash TEXT NOT NULL,
                    state TEXT NOT NULL,
                    status_code INTEGER,
                    response_json TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )

    def _connect(self) -> sqlite3.Connection:
        db = sqlite3.connect(self.path, timeout=10.0)
        db.execute("PRAGMA journal_mode=WAL")
        db.execute("PRAGMA synchronous=FULL")
        return db

    @staticmethod
    def _now() -> str:
        return datetime.now().astimezone().isoformat(timespec="seconds")

    def claim(self, key: str, request_hash: str) -> tuple[str, int | None, dict[str, Any] | None]:
        with self._lock, self._connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute(
                "SELECT request_hash, state, status_code, response_json FROM actions WHERE key=?",
                (key,),
            ).fetchone()
            if row is None:
                now = self._now()
                db.execute(
                    "INSERT INTO actions VALUES (?, ?, 'RUNNING', NULL, NULL, ?, ?)",
                    (key, request_hash, now, now),
                )
                db.commit()
                return "NEW", None, None
            stored_hash, state, status_code, response_json = row
            if stored_hash != request_hash:
                return "CONFLICT", None, None
            if state == "RUNNING":
                return "RUNNING", None, None
            body = json.loads(response_json) if response_json else {}
            return "REPLAY", int(status_code), body

    def finish(self, key: str, state: str, status_code: int, body: dict[str, Any]) -> None:
        if state not in {"SUCCEEDED", "FAILED"}:
            raise ValueError(f"非法终态: {state}")
        encoded = json.dumps(body, ensure_ascii=False, sort_keys=True)
        with self._lock, self._connect() as db:
            cursor = db.execute(
                """
                UPDATE actions
                SET state=?, status_code=?, response_json=?, updated_at=?
                WHERE key=? AND state='RUNNING'
                """,
                (state, int(status_code), encoded, self._now(), key),
            )
            if cursor.rowcount != 1:
                raise RuntimeError(f"幂等动作{key}不存在或已结束")

