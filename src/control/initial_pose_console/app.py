from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field

from hardware.camera_manager import CameraManager
from hardware.robot_manager import RobotManager
from services.pose_store import PoseStore
from services.preset_registry import PresetRegistry


ROOT = Path(__file__).resolve().parent
with (ROOT / "control_config.yaml").open("r", encoding="utf-8") as handle:
    CONFIG = yaml.safe_load(handle)

camera = CameraManager(CONFIG)
robot = RobotManager(CONFIG)
poses = PoseStore(ROOT / "config" / "poses")
REGISTRY_PATH = ROOT.parent / "grasp_pipeline" / "config" / "preset_poses.yaml"
registry = PresetRegistry(REGISTRY_PATH, REGISTRY_PATH.parent / "backups")


@asynccontextmanager
async def lifespan(_app: FastAPI):
    try:
        yield
    finally:
        camera.stop()
        robot.close()


app = FastAPI(title="WRC 整体预设姿态管理台", version="2.0.0", lifespan=lifespan)


class ConfirmedRequest(BaseModel):
    confirmed: bool = False


class ArmMoveRequest(ConfirmedRequest):
    pose: list[float] = Field(min_length=6, max_length=6)
    speed: float = 5


class HeadMoveRequest(ConfirmedRequest):
    yaw: int
    pitch: int


class LiftMoveRequest(ConfirmedRequest):
    height: int
    speed: int = 10


class CreateRegistryPresetRequest(BaseModel):
    name: str
    description: str


class ApplyComponentsRequest(BaseModel):
    components: list[str]


def result(data: Any = None, message: str = "成功") -> dict[str, Any]:
    return {"ok": True, "message": message, "data": data}


def call(action, *args, **kwargs):
    try:
        return action(*args, **kwargs)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


async def async_call(action, *args, **kwargs):
    try:
        return await asyncio.to_thread(action, *args, **kwargs)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.get("/")
def index():
    return FileResponse(ROOT / "static" / "index.html")


@app.get("/static/{name}")
def static_file(name: str):
    if name not in {"app.js", "style.css"}:
        raise HTTPException(status_code=404, detail="not found")
    return FileResponse(ROOT / "static" / name)


@app.get("/api/status")
def status():
    return result({
        "camera": camera.status(),
        "robot": robot.status(),
        "presets": poses.list_all(),
        "pose_registry": call(registry.all),
    })


@app.get("/api/pose-registry")
def get_pose_registry():
    return result(call(registry.all))


@app.post("/api/pose-registry")
def create_registry_preset(request: CreateRegistryPresetRequest):
    data = call(registry.create, request.name, request.description)
    return result(data, "预设已创建")


@app.post("/api/pose-registry/{name}/capture/{component}")
def capture_registry_component(name: str, component: str):
    if component == "head":
        value = call(robot.get_actual_head)
    elif component == "torso":
        value = call(robot.get_actual_lift_height)
    elif component == "left_arm":
        value = call(robot.get_actual_arm_pose, "left")
    elif component == "right_arm":
        value = call(robot.get_actual_arm_pose, "right")
    else:
        raise HTTPException(status_code=400, detail="未知部件")
    preset = call(registry.update_component, name, component, value)
    return result({"component": component, "actual": value, "preset": preset}, "实际状态已更新到YAML")


@app.put("/api/pose-registry/{name}/apply-components")
def update_registry_apply_components(name: str, request: ApplyComponentsRequest):
    data = call(registry.update_apply_components, name, request.components)
    return result(data, "执行范围已更新")


@app.post("/api/camera/{name}/activate")
def activate_camera(name: str):
    return result(call(camera.switch, name), f"已切换到{name}相机")


@app.post("/api/camera/stop")
def stop_camera():
    return result(call(camera.stop), "相机已释放")


@app.get("/api/camera/video")
def video():
    return StreamingResponse(camera.mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/api/camera/snapshot")
def snapshot():
    frame = camera.latest_frame()
    if frame is None:
        raise HTTPException(status_code=409, detail="当前没有可保存的相机帧")
    active = camera.status()["active"] or "camera"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = ROOT / "snapshots" / f"{active}_{stamp}.jpg"
    if not cv2.imwrite(str(path), frame):
        raise HTTPException(status_code=500, detail="图像写入失败")
    return result({"path": str(path)}, "快照保存成功")


@app.post("/api/arm/{side}/connect")
def connect_arm(side: str):
    return result(call(robot.connect_arm, side), f"{side}臂已连接")


@app.post("/api/arm/{side}/disconnect")
def disconnect_arm(side: str):
    return result(call(robot.disconnect_arm, side), f"{side}臂已断开")


@app.post("/api/head/connect")
def connect_head():
    return result(call(robot.connect_head), "头部已连接")


@app.post("/api/head/disconnect")
def disconnect_head():
    return result(call(robot.disconnect_head), "头部已断开")


@app.post("/api/arm/{side}/move")
async def move_arm(side: str, request: ArmMoveRequest):
    data = await async_call(robot.move_arm_pose, side, request.pose, request.speed, request.confirmed)
    return result(data, f"{side}臂运动完成")


@app.post("/api/head/move")
async def move_head(request: HeadMoveRequest):
    data = await async_call(robot.move_head, request.yaw, request.pitch, request.confirmed)
    return result(data, "头部运动完成")


@app.post("/api/lift/move")
async def move_lift(request: LiftMoveRequest):
    data = await async_call(robot.move_lift, request.height, request.speed, request.confirmed)
    return result(data, "升降柱运动完成")


@app.post("/api/motion/stop")
def stop_motion():
    return result(call(robot.stop_all), "停止命令已发送")


@app.get("/api/preset/{name}")
def get_preset(name: str):
    data = call(poses.load, name)
    if data is None:
        raise HTTPException(status_code=404, detail="该预设尚未保存")
    return result(data)


@app.post("/api/preset/{name}/save")
def save_preset(name: str):
    if name in {"left", "right"}:
        pose = call(robot.get_actual_arm_pose, name)
        payload = {
            "frame": f"{name}_arm_base",
            "target": f"{name}_gripper",
            "units": {"position": "m", "orientation": "rad"},
            "pose_6d": pose,
            "pose_order": ["x", "y", "z", "rx", "ry", "rz"],
            "source": "actual_robot_state",
        }
    elif name == "head":
        head = call(robot.get_actual_head)
        lift_state = robot.status()["lift"].get("state")
        payload = {
            "head": head,
            "lift_height": None if lift_state is None else lift_state.get("height"),
            "units": {"yaw_pitch": "servo_value", "lift_height": "mm"},
            "source": "actual_robot_state",
        }
    else:
        raise HTTPException(status_code=400, detail="未知预设")
    path = call(poses.save, name, payload)
    return result({"path": str(path), "preset": poses.load(name)}, "实际姿态已保存")


@app.post("/api/preset/{name}/move")
async def move_preset(name: str, request: ConfirmedRequest):
    preset = call(poses.load, name)
    if preset is None:
        raise HTTPException(status_code=404, detail="该预设尚未保存")
    if name in {"left", "right"}:
        data = await async_call(
            robot.move_arm_pose,
            name,
            preset["pose_6d"],
            CONFIG["motion"]["default_speed"],
            request.confirmed,
        )
    elif name == "head":
        data = await async_call(
            robot.move_head,
            preset["head"]["yaw"],
            preset["head"]["pitch"],
            request.confirmed,
        )
    else:
        raise HTTPException(status_code=400, detail="未知预设")
    return result(data, "已到达保存姿态")


if __name__ == "__main__":
    server = CONFIG["server"]
    uvicorn.run(app, host=server["host"], port=int(server["port"]), log_level="info")
