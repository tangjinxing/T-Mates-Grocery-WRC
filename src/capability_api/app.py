#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

import uvicorn
from fastapi import FastAPI, Header, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from services.grasp_planner import GraspExecutionError, GraspPlanner, GraspPlanningError
from services.gripper_initializer import GripperInitializer
from services.idempotency_store import IdempotencyStore
from services.motion_coordinator import MotionCoordinator
from services.motion_parameters import MotionParameters
from services.pose_executor import PoseExecutor
from services.pose_registry import PoseRegistry, PoseRegistryError
from services.release_executor import (
    ReleaseExecutionError,
    ReleaseExecutor,
    ReleasePlanningError,
)


ROOT = Path(__file__).resolve().parent
POSE_CONFIG = ROOT / "config" / "capability_poses.yaml"
IDEMPOTENCY_DB = ROOT / "runtime" / "idempotency.sqlite3"
HARDWARE_INIT_CONFIG = ROOT / "config" / "hardware_init.yaml"
MOTION_PARAMETERS_CONFIG = ROOT / "config" / "motion_parameters.yaml"
registry = PoseRegistry(POSE_CONFIG)
coordinator = MotionCoordinator()
motion_parameters = MotionParameters(MOTION_PARAMETERS_CONFIG)
executor = PoseExecutor(coordinator, motion_parameters)
grasp_planner = GraspPlanner(coordinator, motion_parameters)
release_executor = ReleaseExecutor(coordinator, POSE_CONFIG, motion_parameters)
gripper_initializer = GripperInitializer(HARDWARE_INIT_CONFIG, coordinator)
idempotency = IdempotencyStore(IDEMPOTENCY_DB)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    # 这是有物理动作的启动初始化：可能设置24V并打开左右夹爪。
    # 禁止放到模块导入阶段，否则测试或OpenAPI导入也会触发硬件。
    gripper_initializer.initialize_all()
    yield


app = FastAPI(title="WRC Capability API", version="0.3.0-live", lifespan=lifespan)


class PrepareRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    pose_id: str = Field(min_length=1, max_length=128)


class GraspRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    pose: list[float] = Field(min_length=6, max_length=6)
    hand: Literal["LEFT", "RIGHT", "left", "right"]
    frame: Literal["camera"] = "camera"
    pose_unit: Literal["mm_rad"] = "mm_rad"
    rotation_order: Literal["xyz", "zyx"] = "zyx"
    execute: bool = False


class ReleaseRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    hand: Literal["LEFT", "RIGHT", "left", "right"]
    execute: bool = False


def error(status_code: int, error_code: str, message: str) -> JSONResponse:
    return JSONResponse(status_code=status_code, content={"error_code": error_code, "message": message})


@app.exception_handler(PoseRegistryError)
async def pose_registry_error_handler(_request, exc: PoseRegistryError):
    status = 404 if exc.error_code == "POSE_NOT_FOUND" else 409
    if exc.error_code in {"CONFIG_NOT_FOUND", "CONFIG_INVALID"}:
        status = 503
    return error(status, exc.error_code, exc.message)


@app.get("/health")
def health():
    try:
        poses = registry.summary()
    except PoseRegistryError as exc:
        return error(503, exc.error_code, exc.message)
    return {
        "status": "READY",
        "mode": "LIVE",
        "pose_config": str(POSE_CONFIG),
        "motion_parameters_config": str(MOTION_PARAMETERS_CONFIG),
        "motion_parameters": motion_parameters.summary(),
        "poses": poses,
    }


@app.get("/pose/health")
def pose_health():
    return health()


@app.get("/manipulation/health")
def manipulation_health():
    grippers = gripper_initializer.snapshot()
    ready = all(
        bool(grippers.get("arms", {}).get(side, {}).get("initialized"))
        for side in ("left", "right")
    )
    body = {
        "status": "READY" if ready else "DEGRADED",
        "mode": "LIVE_GUARDED",
        "default_execute": False,
        "grippers": grippers,
    }
    return JSONResponse(status_code=200 if ready else 503, content=body)


@app.post("/pose/prepare")
def prepare_pose(
    request: PrepareRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
):
    if idempotency_key is None or not idempotency_key.strip():
        return error(400, "IDEMPOTENCY_KEY_REQUIRED", "物理动作接口必须提供Idempotency-Key")
    key = idempotency_key.strip()
    if len(key) > 256:
        return error(400, "IDEMPOTENCY_KEY_INVALID", "Idempotency-Key长度不能超过256")
    request_hash = hashlib.sha256(
        json.dumps(request.model_dump(), ensure_ascii=False, sort_keys=True).encode("utf-8")
    ).hexdigest()
    state, cached_code, cached_body = idempotency.claim(key, request_hash)
    if state == "CONFLICT":
        return error(409, "IDEMPOTENCY_KEY_CONFLICT", "相同Idempotency-Key不能用于不同请求")
    if state == "RUNNING":
        return error(409, "EXECUTION_FAILED", "该动作正在执行或服务曾在执行期间中断，禁止自动重试")
    if state == "REPLAY":
        return JSONResponse(status_code=int(cached_code), content=dict(cached_body))

    try:
        resolved = registry.resolve(request.pose_id)
        executor.execute(resolved)
        body = {"status": "SUCCEEDED"}
        status_code = 200
    except Exception as exc:
        status_code = 500
        message = exc.message if isinstance(exc, PoseRegistryError) else str(exc)
        body = {"error_code": "EXECUTION_FAILED", "message": message}

    idempotency.finish(
        key, "SUCCEEDED" if status_code == 200 else "FAILED", status_code, body
    )
    return JSONResponse(status_code=status_code, content=body)


@app.post("/manipulation/grasp")
def plan_grasp(
    request: GraspRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
):
    if idempotency_key is None or not idempotency_key.strip():
        return error(400, "IDEMPOTENCY_KEY_REQUIRED", "抓取接口必须提供Idempotency-Key")
    raw_key = idempotency_key.strip()
    if len(raw_key) > 256:
        return error(400, "IDEMPOTENCY_KEY_INVALID", "Idempotency-Key长度不能超过256")
    if request.execute and not gripper_initializer.is_ready(request.hand):
        return error(
            503,
            "GRIPPER_NOT_INITIALIZED",
            f"{request.hand.upper()}夹爪启动初始化未通过，请检查/manipulation/health",
        )

    # 与姿态准备共用数据库，但使用独立命名空间，避免不同能力偶然撞键。
    key = "grasp:" + raw_key
    request_hash = hashlib.sha256(
        json.dumps(request.model_dump(), ensure_ascii=False, sort_keys=True).encode("utf-8")
    ).hexdigest()
    state, cached_code, cached_body = idempotency.claim(key, request_hash)
    if state == "CONFLICT":
        return error(409, "IDEMPOTENCY_KEY_CONFLICT", "相同Idempotency-Key不能用于不同请求")
    if state == "RUNNING":
        return error(409, "EXECUTION_FAILED", "该抓取规划正在执行或曾异常中断，禁止自动重试")
    if state == "REPLAY":
        return JSONResponse(status_code=int(cached_code), content=dict(cached_body))

    try:
        body = grasp_planner.plan(
            request.hand, request.pose, request.rotation_order, request.execute
        )
        status_code = 200
    except GraspPlanningError as exc:
        status_code = 422
        body = {
            "error_code": "UNREACHABLE",
            "message": str(exc),
            "status": "UNREACHABLE",
            "reachable": False,
            "executed": False,
        }
    except GraspExecutionError as exc:
        status_code = 500
        body = {
            "error_code": "EXECUTION_FAILED",
            "message": str(exc),
            "status": "FAILED",
            "executed": True,
        }
    except Exception as exc:
        status_code = 500
        body = {
            "error_code": "EXECUTION_FAILED",
            "message": str(exc),
            "executed": False,
        }

    idempotency.finish(
        key, "SUCCEEDED" if status_code == 200 else "FAILED", status_code, body
    )
    return JSONResponse(status_code=status_code, content=body)


@app.post("/manipulation/release")
def release_object(
    request: ReleaseRequest,
    idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
):
    if idempotency_key is None or not idempotency_key.strip():
        return error(400, "IDEMPOTENCY_KEY_REQUIRED", "放置接口必须提供Idempotency-Key")
    raw_key = idempotency_key.strip()
    if len(raw_key) > 256:
        return error(400, "IDEMPOTENCY_KEY_INVALID", "Idempotency-Key长度不能超过256")
    if request.execute and not gripper_initializer.is_ready(request.hand):
        return error(
            503,
            "GRIPPER_NOT_INITIALIZED",
            f"{request.hand.upper()}夹爪启动初始化未通过，请检查/manipulation/health",
        )

    key = "release:" + raw_key
    request_hash = hashlib.sha256(
        json.dumps(request.model_dump(), ensure_ascii=False, sort_keys=True).encode("utf-8")
    ).hexdigest()
    state, cached_code, cached_body = idempotency.claim(key, request_hash)
    if state == "CONFLICT":
        return error(409, "IDEMPOTENCY_KEY_CONFLICT", "相同Idempotency-Key不能用于不同请求")
    if state == "RUNNING":
        return error(409, "EXECUTION_FAILED", "该放置动作正在执行或曾异常中断，禁止自动重试")
    if state == "REPLAY":
        return JSONResponse(status_code=int(cached_code), content=dict(cached_body))

    try:
        body = release_executor.run(request.hand, request.execute)
        status_code = 200
    except ReleasePlanningError as exc:
        status_code = 422
        body = {
            "error_code": "UNREACHABLE",
            "message": str(exc),
            "status": "UNREACHABLE",
            "reachable": False,
            "executed": False,
        }
    except ReleaseExecutionError as exc:
        status_code = 500
        body = {
            "error_code": "EXECUTION_FAILED",
            "message": str(exc),
            "status": "FAILED",
            "executed": True,
        }
    except Exception as exc:
        status_code = 500
        body = {
            "error_code": "EXECUTION_FAILED",
            "message": str(exc),
            "status": "FAILED",
            "executed": bool(request.execute),
        }

    idempotency.finish(
        key, "SUCCEEDED" if status_code == 200 else "FAILED", status_code, body
    )
    return JSONResponse(status_code=status_code, content=body)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8099, log_level="info")
