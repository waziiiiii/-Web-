from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict
# Use mock sim for rapid frontend development while pybullet compiles
from robot_sim_mock import RobotSimMock as RobotSim
# from robot_sim import RobotSim  # Switch to real pybullet once installed
from controller import RobotController
import asyncio
from ws_manager import WSManager

router = APIRouter()
# Use mock (fast) or DIRECT (headless) mode
sim = RobotSim(gui=False)
controller = RobotController(sim)
manager = WSManager()


@router.get("/status")
def status():
    return {"joints": sim.get_joint_states(), "end_effector": sim.get_end_effector_pose()}


@router.post("/set_joints")
def set_joints(joint_angles: Dict[int, float]):
    sim.set_joint_angles_list(joint_angles)
    return {"ok": True}


@router.websocket("/ws/images")
async def ws_images(websocket: WebSocket):
    await manager.connect_image(websocket)
    try:
        # keep connection open; images are pushed from background task
        while True:
            await asyncio.sleep(3600)
    except WebSocketDisconnect:
        manager.disconnect(websocket)


@router.websocket("/ws/control")
async def ws_control(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_json()
            cmd_type = data.get("type")
            if cmd_type == "set_joints":
                joints = data.get("joints", {})
                sim.set_joint_angles_list(joints)
                await websocket.send_json({"ok": True})
            elif cmd_type == "move_to":
                pos = data.get("position")
                controller.move_to_position(pos)
                await websocket.send_json({"ok": True})
            else:
                await websocket.send_json({"error": "unknown command"})
    except WebSocketDisconnect:
        return


async def start_background_tasks():
    image_interval = 1.0 / 10.0
    last_image_time = asyncio.get_event_loop().time()
    try:
        while True:
            sim.step()
            now = asyncio.get_event_loop().time()
            if now - last_image_time >= image_interval:
                try:
                    img_b64 = sim.get_camera_image_base64()
                    await manager.broadcast_image(img_b64)
                except Exception:
                    pass
                last_image_time = now
            await asyncio.sleep(1.0 / 240.0)
    except asyncio.CancelledError:
        return
