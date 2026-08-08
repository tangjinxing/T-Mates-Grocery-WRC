"""Pure facade orchestrating state machine + adapter (no ROS dependency)."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from retail_nav_bridge.adapters.base import (
    AdapterResult,
    HealthInfo,
    NavAdapter,
    NavGoal,
    Pose2D,
)
from retail_nav_bridge.constants import (
    MAPPING_DONE,
    MAPPING_NONE,
    MAPPING_SCANNING,
    ErrorCode,
    EventType,
    Mode,
    NavState,
    ResultCode,
)
from retail_nav_bridge.state_machine import FacadeSnapshot, NavStateMachine
from retail_nav_bridge.stations import Station, StationCatalog


@dataclass
class FacadeEvent:
    type: str
    request_id: str = ""
    station_id: str = ""
    error_code: int = 0
    message: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


EventCallback = Callable[[FacadeEvent], None]


@dataclass
class CommandResult:
    error_code: int = ErrorCode.OK
    error_msg: str = ""
    nav_state: str = NavState.UNREADY.value
    session_id: str = ""
    map_id: str = ""
    map_name: str = ""
    localized: bool = False
    health: Optional[HealthInfo] = None
    stations: List[Station] = field(default_factory=list)
    station_count: int = 0
    stations_file: str = ""
    package_path: str = ""
    bmap_ok: bool = False
    exported_files: List[str] = field(default_factory=list)
    format_status: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.error_code == ErrorCode.OK


class NavFacade:
    def __init__(
        self,
        adapter: NavAdapter,
        catalog: StationCatalog,
        *,
        on_event: Optional[EventCallback] = None,
        stations_file: str = "",
    ) -> None:
        self.adapter = adapter
        self.catalog = catalog
        self.on_event = on_event
        self.stations_file = stations_file
        self.sm = NavStateMachine()
        self.sm.snap.map_name = catalog.map_name
        self._active_goal: Optional[NavGoal] = None
        self._nav_t0 = 0.0
        self._pre_hold_state: NavState = NavState.READY

    def replace_catalog(self, catalog: StationCatalog) -> None:
        """Hot-swap in-memory station catalog (and adapter copy if present)."""
        self.catalog = catalog
        if catalog.map_name:
            self.sm.snap.map_name = catalog.map_name
        # S2NavAdapter keeps its own catalog reference for vendor_target_id lookups.
        if hasattr(self.adapter, "_catalog"):
            setattr(self.adapter, "_catalog", catalog)

    @property
    def snapshot(self) -> FacadeSnapshot:
        return self.sm.snap

    def _emit(self, event: FacadeEvent) -> None:
        if self.on_event:
            self.on_event(event)

    def _ok(self, **kwargs) -> CommandResult:
        return CommandResult(
            error_code=ErrorCode.OK,
            nav_state=self.sm.state.value,
            localized=self.sm.snap.localized,
            **kwargs,
        )

    def _fail(self, code: int, msg: str) -> CommandResult:
        return CommandResult(
            error_code=code,
            error_msg=msg,
            nav_state=self.sm.state.value,
            localized=self.sm.snap.localized,
        )

    def get_pose(self) -> Pose2D:
        return self.adapter.get_pose()

    def init(self) -> CommandResult:
        health = self.adapter.get_health()
        if not health.ready:
            return self._fail(ErrorCode.NOT_READY, health.detail or "adapter not ready")
        ok, code, msg = self.sm.transition(NavState.IDLE)
        if not ok:
            # Allow re-init from IDLE/READY
            if self.sm.state in (NavState.IDLE, NavState.READY):
                return self._ok(health=health)
            return self._fail(code, msg)
        self.sm.snap.mode = Mode.NONE
        self.sm.snap.mapping_status = MAPPING_NONE
        return self._ok(health=health)

    def get_health(self) -> CommandResult:
        return self._ok(health=self.adapter.get_health())

    def clear_error(self) -> CommandResult:
        ok, code, msg = self.sm.clear_error(
            NavState.READY if self.sm.snap.localized else NavState.IDLE
        )
        if not ok:
            return self._fail(code, msg)
        return self._ok()

    def start_mapping(self, map_name: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.IDLE, NavState.READY)
        if not ok:
            return self._fail(code, msg)
        if self.sm.snap.holding:
            return self._fail(ErrorCode.INVALID_STATE, "holding")
        result = self.adapter.start_mapping(map_name)
        if not result.ok:
            return self._fail(result.error_code, result.error_msg)
        self.sm.transition(NavState.MAPPING)
        self.sm.snap.mode = Mode.MAPPING
        self.sm.snap.mapping_status = MAPPING_SCANNING
        self.sm.snap.localized = False
        self.sm.snap.map_name = result.map_name or map_name
        return self._ok(session_id=result.session_id)

    def begin_stop_mapping(self, map_name: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.MAPPING)
        if not ok:
            return self._fail(code, msg)
        self.sm.transition(NavState.SAVING)
        self.sm.snap.progress = 0.0
        if map_name:
            self.sm.snap.map_name = map_name
        return self._ok()

    def tick_stop_mapping(self, dt: float = 0.1) -> AdapterResult:
        result = self.adapter.tick_stop_mapping(self.sm.snap.map_name, dt=dt)
        self.sm.snap.progress = result.progress
        if result.progress >= 1.0 and result.ok:
            self.sm.snap.map_id = result.map_id
            self.sm.snap.map_name = result.map_name or self.sm.snap.map_name
            self.sm.snap.mapping_status = MAPPING_DONE
            self.sm.snap.mode = Mode.NONE
            self.sm.transition(NavState.IDLE)
            self._emit(
                FacadeEvent(
                    type=EventType.MAPPING_DONE.value,
                    message=f"map saved: {self.sm.snap.map_name}",
                    error_code=0,
                )
            )
        elif not result.ok:
            self.sm.set_error(result.error_code, result.error_msg)
            self._emit(
                FacadeEvent(
                    type=EventType.ERROR.value,
                    error_code=result.error_code,
                    message=result.error_msg,
                )
            )
        return result

    def cancel_mapping(self, reason: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.MAPPING, NavState.SAVING)
        if not ok:
            return self._fail(code, msg)
        self.adapter.cancel_mapping(reason)
        self.sm.snap.mapping_status = MAPPING_NONE
        self.sm.snap.mode = Mode.NONE
        self.sm.snap.progress = 0.0
        self.sm.transition(NavState.IDLE)
        return self._ok()

    def load_map(self, map_id: str = "", map_name: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(
            NavState.IDLE, NavState.READY, NavState.UNREADY
        )
        if not ok:
            return self._fail(code, msg)
        result = self.adapter.load_map(map_id=map_id, map_name=map_name)
        if not result.ok:
            return self._fail(result.error_code, result.error_msg)
        self.sm.snap.map_id = result.map_id or map_id
        self.sm.snap.map_name = result.map_name or map_name or self.catalog.map_name
        if self.sm.state == NavState.UNREADY:
            self.sm.transition(NavState.IDLE)
        return self._ok(map_id=self.sm.snap.map_id)

    def relocate(
        self,
        *,
        station_id: str = "",
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.IDLE, NavState.READY)
        if not ok:
            return self._fail(code, msg)
        station = None
        if station_id:
            station = self.catalog.get(station_id)
            if station is None:
                return self._fail(ErrorCode.UNKNOWN_STATION, f"unknown station: {station_id}")
        self.sm.transition(NavState.LOCALIZING)
        result = self.adapter.relocate(station=station, x=x, y=y, yaw=yaw)
        if not result.ok:
            self.sm.set_error(result.error_code, result.error_msg)
            return self._fail(result.error_code, result.error_msg)
        self.sm.snap.localized = True
        self.sm.snap.mode = Mode.NAVIGATION
        self.sm.transition(NavState.READY)
        pose = result.pose or self.adapter.get_pose()
        self._emit(
            FacadeEvent(
                type=EventType.RELOC_OK.value,
                station_id=station_id,
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw,
            )
        )
        return self._ok()

    def list_stations(self) -> CommandResult:
        return self._ok(stations=self.catalog.list(), station_count=len(self.catalog.stations))

    def sync_stations(
        self,
        package_path: str,
        *,
        target_stations_file: str = "",
    ) -> CommandResult:
        """Sync stations from a map package (or stations.yaml) into retail_stations.yaml.

        Does NOT load/switch maps — call LoadMap separately. Refuses while navigating /
        mapping / saving so mid-mission catalogs stay stable.
        """
        from retail_nav_bridge.map_package import sync_stations_to_file

        ok, code, msg = self.sm.require_states(
            NavState.UNREADY,
            NavState.IDLE,
            NavState.READY,
            NavState.HOLDING,
            NavState.ERROR,
        )
        if not ok:
            return self._fail(code, msg)
        target = (target_stations_file or self.stations_file or "").strip()
        if not target:
            return self._fail(ErrorCode.NOT_READY, "stations_file not configured")
        if not (package_path or "").strip():
            return self._fail(ErrorCode.NOT_READY, "package_path is empty")
        try:
            catalog = sync_stations_to_file(package_path, target)
        except FileNotFoundError as exc:
            return self._fail(ErrorCode.NOT_READY, str(exc))
        except Exception as exc:  # noqa: BLE001
            return self._fail(ErrorCode.VENDOR, f"sync stations failed: {exc}")
        self.replace_catalog(catalog)
        self.stations_file = target
        return self._ok(
            station_count=len(catalog.stations),
            stations_file=target,
            map_name=catalog.map_name,
            stations=catalog.list(),
        )

    def export_map_package(
        self,
        output_dir: str,
        *,
        map_id: str = "",
        map_name: str = "",
        map_path: str = "",
        copy_map_files: bool = False,
        notes: str = "",
    ) -> CommandResult:
        """Export current in-memory stations (+ map metadata) as a map package."""
        from retail_nav_bridge.map_package import export_map_package

        if not (output_dir or "").strip():
            return self._fail(ErrorCode.NOT_READY, "output_dir is empty")
        try:
            out = export_map_package(
                output_dir,
                self.catalog,
                map_id=map_id or self.sm.snap.map_id,
                map_name=map_name or self.sm.snap.map_name or self.catalog.map_name,
                map_path=map_path,
                copy_map_files=copy_map_files,
                notes=notes,
            )
        except Exception as exc:  # noqa: BLE001
            return self._fail(ErrorCode.VENDOR, f"export map package failed: {exc}")
        return self._ok(
            package_path=str(out),
            station_count=len(self.catalog.stations),
            map_name=self.catalog.map_name,
            map_id=map_id or self.sm.snap.map_id,
        )

    def export_vendor_map_package(
        self,
        output_root: str,
        *,
        map_id: str = "",
        map_name: str = "",
        notes: str = "",
    ) -> CommandResult:
        """Export current TX-S2 map artifacts without touching station YAML."""
        from retail_nav_bridge.map_package import (
            VENDOR_MAP_FILES,
            export_vendor_map_package,
        )

        export_data = getattr(self.adapter, "export_vendor_map_data", None)
        if not callable(export_data):
            return self._fail(
                ErrorCode.NOT_READY,
                "vendor map export requires adapter:=s2",
            )
        if not (output_root or "").strip():
            return self._fail(ErrorCode.NOT_READY, "vendor_map_export_dir not configured")

        resolved_map_id = map_id or self.sm.snap.map_id
        resolved_map_name = map_name or self.sm.snap.map_name or self.catalog.map_name
        try:
            data_by_type = export_data()
            package_path, statuses = export_vendor_map_package(
                output_root,
                data_by_type,
                map_id=resolved_map_id,
                map_name=resolved_map_name,
                notes=notes,
            )
        except Exception as exc:  # noqa: BLE001
            return self._fail(ErrorCode.VENDOR, f"vendor map export failed: {exc}")

        exported_files = [
            filename
            for map_type, filename in VENDOR_MAP_FILES.items()
            if statuses.get(map_type) == "exported"
        ]
        format_status = [
            f"{VENDOR_MAP_FILES[map_type]}={statuses.get(map_type, 'unavailable')}"
            for map_type in sorted(VENDOR_MAP_FILES)
        ]
        return self._ok(
            package_path=str(package_path),
            map_id=resolved_map_id,
            map_name=resolved_map_name,
            bmap_ok=statuses.get(0) == "exported",
            exported_files=exported_files,
            format_status=format_status,
        )

    def start_go_to_station(
        self,
        station_id: str,
        *,
        request_id: str = "",
        max_speed: float = 0.0,
        timeout_sec: float = 0.0,
    ) -> CommandResult:
        station = self.catalog.get(station_id)
        if station is None:
            return self._fail(ErrorCode.UNKNOWN_STATION, f"unknown station: {station_id}")
        goal = NavGoal(
            request_id=request_id or str(uuid.uuid4()),
            station_id=station.station_id,
            x=station.x,
            y=station.y,
            yaw=station.yaw,
            arrive_tol_m=station.arrive_tol_m,
            yaw_tol_rad=station.yaw_tol_rad,
            max_speed=max_speed or station.max_speed,
            timeout_sec=timeout_sec or 120.0,
        )
        return self._start_nav(goal)

    def start_go_to_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        request_id: str = "",
        arrive_tol_m: float = 0.08,
        yaw_tol_rad: float = 0.17,
        max_speed: float = 0.5,
        timeout_sec: float = 120.0,
    ) -> CommandResult:
        goal = NavGoal(
            request_id=request_id or str(uuid.uuid4()),
            x=x,
            y=y,
            yaw=yaw,
            arrive_tol_m=arrive_tol_m,
            yaw_tol_rad=yaw_tol_rad,
            max_speed=max_speed,
            timeout_sec=timeout_sec,
        )
        return self._start_nav(goal)

    def start_move_relative(
        self,
        dx: float,
        dy: float,
        dyaw: float,
        *,
        request_id: str = "",
        timeout_sec: float = 30.0,
    ) -> CommandResult:
        goal = NavGoal(
            request_id=request_id or str(uuid.uuid4()),
            x=dx,
            y=dy,
            yaw=dyaw,
            arrive_tol_m=0.03,
            yaw_tol_rad=0.05,
            max_speed=0.2,
            timeout_sec=timeout_sec,
            relative=True,
        )
        return self._start_nav(goal)

    def _start_nav(self, goal: NavGoal) -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.READY, NavState.ARRIVED)
        if not ok:
            return self._fail(code, msg)
        if self.sm.snap.holding:
            return self._fail(ErrorCode.INVALID_STATE, "holding; ReleaseHold first")
        result = self.adapter.start_nav(goal)
        if not result.ok:
            return self._fail(result.error_code, result.error_msg)
        self._active_goal = goal
        self._nav_t0 = time.monotonic()
        self.sm.snap.request_id = goal.request_id
        self.sm.snap.active_station_id = goal.station_id
        self.sm.snap.progress = 0.0
        self.sm.snap.mode = Mode.NAVIGATION
        self.sm.transition(NavState.NAVIGATING)
        return self._ok()

    def tick_nav(self, dt: float = 0.1) -> AdapterResult:
        if self.sm.state not in (NavState.NAVIGATING, NavState.PAUSED):
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg=f"not navigating ({self.sm.state.value})",
                result_code=ResultCode.ERROR.value,
            )
        if self.sm.state == NavState.PAUSED:
            return AdapterResult(progress=self.sm.snap.progress, pose=self.adapter.get_pose())

        result = self.adapter.tick_nav(dt=dt)
        self.sm.snap.progress = result.progress
        if result.result_code == ResultCode.ARRIVED.value:
            self.sm.transition(NavState.ARRIVED)
            pose = result.pose or self.adapter.get_pose()
            self._emit(
                FacadeEvent(
                    type=EventType.ARRIVED.value,
                    request_id=self.sm.snap.request_id,
                    station_id=self.sm.snap.active_station_id,
                    x=pose.x,
                    y=pose.y,
                    yaw=pose.yaw,
                )
            )
            self.sm.transition(NavState.READY)
            self.sm.snap.progress = 1.0
            self._active_goal = None
        elif result.result_code == ResultCode.BLOCKED.value:
            self.sm.set_error(result.error_code, result.error_msg)
            self._emit(
                FacadeEvent(
                    type=EventType.BLOCKED.value,
                    request_id=self.sm.snap.request_id,
                    station_id=self.sm.snap.active_station_id,
                    error_code=result.error_code,
                    message=result.error_msg,
                )
            )
            self._active_goal = None
        elif result.result_code == ResultCode.TIMEOUT.value:
            self.sm.set_error(result.error_code, result.error_msg)
            self._emit(
                FacadeEvent(
                    type=EventType.ERROR.value,
                    request_id=self.sm.snap.request_id,
                    error_code=result.error_code,
                    message=result.error_msg,
                )
            )
            self._active_goal = None
        elif not result.ok and result.result_code:
            self.sm.set_error(result.error_code, result.error_msg)
            self._emit(
                FacadeEvent(
                    type=EventType.ERROR.value,
                    request_id=self.sm.snap.request_id,
                    error_code=result.error_code,
                    message=result.error_msg,
                )
            )
            self._active_goal = None
        return result

    def cancel_nav(self, request_id: str = "", reason: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(
            NavState.NAVIGATING, NavState.PAUSED, NavState.ARRIVED
        )
        if not ok:
            return self._fail(code, msg)
        if request_id and self.sm.snap.request_id and request_id != self.sm.snap.request_id:
            return self._fail(ErrorCode.INVALID_STATE, "request_id mismatch")
        self.adapter.cancel_nav(reason)
        rid = self.sm.snap.request_id
        sid = self.sm.snap.active_station_id
        self._active_goal = None
        self.sm.snap.request_id = ""
        self.sm.snap.active_station_id = ""
        self.sm.snap.progress = 0.0
        self.sm.transition(NavState.READY)
        self._emit(
            FacadeEvent(
                type=EventType.CANCELLED.value,
                request_id=rid,
                station_id=sid,
                message=reason,
                error_code=ErrorCode.CANCELLED,
            )
        )
        return self._ok()

    def pause(self, reason: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.NAVIGATING)
        if not ok:
            return self._fail(code, msg)
        self.adapter.pause_nav()
        self.sm.transition(NavState.PAUSED)
        self.sm.snap.error_msg = reason
        return self._ok()

    def resume(self) -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.PAUSED)
        if not ok:
            return self._fail(code, msg)
        self.adapter.resume_nav()
        self.sm.transition(NavState.NAVIGATING)
        return self._ok()

    def hold(self, reason: str = "") -> CommandResult:
        ok, code, msg = self.sm.require_states(
            NavState.READY, NavState.ARRIVED, NavState.IDLE
        )
        if not ok:
            return self._fail(code, msg)
        self._pre_hold_state = self.sm.state if self.sm.state != NavState.ARRIVED else NavState.READY
        self.adapter.hold(reason)
        self.sm.snap.hold_reason = reason
        self.sm.transition(NavState.HOLDING)
        return self._ok()

    def release_hold(self) -> CommandResult:
        ok, code, msg = self.sm.require_states(NavState.HOLDING)
        if not ok:
            return self._fail(code, msg)
        self.adapter.release_hold()
        self.sm.snap.hold_reason = ""
        target = self._pre_hold_state if self.sm.snap.localized else NavState.IDLE
        if target not in (NavState.READY, NavState.IDLE):
            target = NavState.READY if self.sm.snap.localized else NavState.IDLE
        self.sm.transition(target)
        return self._ok()

    def elapsed_nav_sec(self) -> float:
        if not self._nav_t0:
            return 0.0
        return time.monotonic() - self._nav_t0
