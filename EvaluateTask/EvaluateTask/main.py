"""FastAPI backend - clean rewrite for 3D visualization + joint control."""

from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Body, FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse

from robot_controller import RobotController
import config as cfg

robot: RobotController | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global robot
    robot = RobotController()
    print(f"[init] Robot: {len(robot._controllable_joints)} controllable joints, {cfg.TARGET_FPS} FPS")
    yield
    robot.shutdown()
    print("[shutdown] Done")


app = FastAPI(title="Robot Arm Simulation", lifespan=lifespan)
STATIC_DIR = Path(__file__).resolve().parent / "static"


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse(
        content=(STATIC_DIR / "index.html").read_text("utf-8"),
        headers={"Cache-Control": "no-cache, no-store, must-revalidate"},
    )


MESH_DIR = Path(__file__).resolve().parent / "autolife_desktop" / "meshes" / "robot_v0_1"


@app.get("/static/{path:path}")
async def static_file(path: str):
    fp = STATIC_DIR / path
    if not fp.exists():
        return HTMLResponse(status_code=404, content="Not found")
    ext = fp.suffix.lower()
    mt = {".js": "application/javascript", ".css": "text/css"}.get(ext, "application/octet-stream")
    return FileResponse(str(fp), media_type=mt, headers={"Cache-Control": "no-cache"})


@app.get("/meshes/{filename:path}")
async def mesh_file(filename: str):
    """Serve STL mesh files for 3D rendering."""
    fp = MESH_DIR / filename
    if not fp.exists():
        return HTMLResponse(status_code=404, content="Mesh not found")
    return FileResponse(str(fp), media_type="application/octet-stream",
                        headers={"Cache-Control": "max-age=3600"})


@app.get("/api/info")
async def api_info():
    return {"joints": robot._controllable_joints, "num_joints": len(robot._controllable_joints)}


@app.get("/api/state")
async def api_state():
    return robot.get_latest_state()


@app.post("/api/joint/{joint_index}")
async def api_set_joint(joint_index: int, data: dict = Body(...)):
    robot.set_joint(joint_index, float(data.get("angle", 0)))
    return {"status": "ok"}


@app.post("/api/ik")
async def api_ik(data: dict = Body(...)):
    """Solve IK and move if successful. Body: {"x":0.3, "y":0.0, "z":0.3, "roll":0, "pitch":0, "yaw":0}"""
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", 0))
    roll = data.get("roll")
    pitch = data.get("pitch")
    yaw = data.get("yaw")
    if roll is not None and pitch is not None and yaw is not None:
        result = robot.apply_ik_and_set(x, y, z, float(roll), float(pitch), float(yaw))
    else:
        result = robot.apply_ik_and_set(x, y, z)
    return result


@app.post("/api/ik_solve")
async def api_ik_solve(data: dict = Body(...)):
    """Solve IK without moving."""
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", 0))
    roll = data.get("roll")
    pitch = data.get("pitch")
    yaw = data.get("yaw")
    if roll is not None and pitch is not None and yaw is not None:
        result = robot.solve_ik(x, y, z, float(roll), float(pitch), float(yaw))
    else:
        result = robot.solve_ik(x, y, z)
    return result


@app.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            raw = await ws.receive_text()
            msg = json.loads(raw)
            t = msg.get("type")
            if t == "set_joint":
                robot.set_joint(int(msg["index"]), float(msg["angle"]))
                await ws.send_json({"status": "ok"})
            elif t == "set_joints":
                robot.set_joints([float(a) for a in msg["angles"]])
                await ws.send_json({"status": "ok"})
            elif t == "ik":
                x = float(msg["x"])
                y = float(msg["y"])
                z = float(msg["z"])
                roll = msg.get("roll")
                pitch = msg.get("pitch")
                yaw = msg.get("yaw")
                if roll is not None and pitch is not None and yaw is not None:
                    result = robot.apply_ik_and_set(x, y, z, float(roll), float(pitch), float(yaw))
                else:
                    result = robot.apply_ik_and_set(x, y, z)
                await ws.send_json(result)
            elif t == "camera_toggle":
                # Legacy compat — camera is now on-demand only
                await ws.send_json({"status": "ok", "camera_active": False})
            elif t == "get_state":
                await ws.send_json(robot.get_latest_state())
            else:
                await ws.send_json({"error": f"unknown: {t}"})
    except (WebSocketDisconnect, Exception):
        pass


@app.get("/api/control_mode")
async def api_get_control_mode():
    """Return the current control mode."""
    return {"mode": robot.get_control_mode()}


@app.post("/api/control_mode")
async def api_set_control_mode(data: dict = Body(...)):
    """Switch control mode. Body: {"mode": "precise"} or {"mode": "motor"}"""
    mode = data.get("mode", "precise")
    robot.set_control_mode(mode)
    return {"status": "ok", "mode": mode}


@app.get("/api/collision")
async def api_collision():
    """Return the current collision state."""
    return robot.get_collision_state()


@app.get("/api/collision/debug")
async def api_collision_debug():
    """Debug: return ALL raw self-contact points before filtering."""
    return robot.get_debug_self_contacts()


@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    """State-only stream at 60 FPS — no camera data, lightweight."""
    await ws.accept()
    interval = 1.0 / cfg.STREAM_FPS
    try:
        while True:
            state = robot.get_latest_state()
            if state:
                await ws.send_json(state)
            await asyncio.sleep(interval)
    except (WebSocketDisconnect, Exception):
        pass


@app.websocket("/ws/camera")
async def ws_camera(ws: WebSocket):
    """Independent camera stream using binary frames — yields to event loop between sends."""
    await ws.accept()
    interval = 1.0 / 10  # Match camera thread rate
    try:
        while True:
            rgb_bytes, depth_bytes = robot.get_latest_camera_bytes()
            if rgb_bytes and depth_bytes:
                # Send as binary: [1-byte type (0=RGB, 1=Depth)] + JPEG data
                # yield between sends to prevent blocking the event loop
                await ws.send_bytes(b'\x00' + rgb_bytes)   # prefix 0x00 = RGB
                await asyncio.sleep(0)  # yield to event loop
                await ws.send_bytes(b'\x01' + depth_bytes) # prefix 0x01 = Depth
                await asyncio.sleep(0)  # yield to event loop
            await asyncio.sleep(interval)
    except (WebSocketDisconnect, Exception):
        pass


@app.websocket("/ws/state_stream")
async def ws_state_stream(ws: WebSocket):
    await ws_stream(ws)


@app.post("/api/workspace/objects")
async def api_update_workspace_objects(data: dict = Body(...)):
    """Update workspace object positions (synced from Three.js drag)."""
    objects = data.get("objects", [])
    robot.set_workspace_objects(objects)
    return {"status": "ok", "count": len(objects)}


@app.post("/api/camera/capture")
async def api_camera_capture(data: dict = Body(None)):
    """Trigger a single-frame capture (non-blocking). Returns immediately.
    Optionally accepts workspace object positions in the request body.
    """
    if data and "objects" in data:
        robot.set_workspace_objects(data["objects"])
    if robot._cam_capturing:
        return {"status": "busy", "message": "Capture already in progress"}
    robot.capture_single_frame()
    return {"status": "started", "message": "Capture thread spawned"}


@app.get("/api/camera/status")
async def api_camera_status():
    """Check if a capture is in progress and whether frame data is available."""
    rgb_bytes, depth_bytes = robot.get_latest_camera_bytes()
    return {
        "capturing": robot._cam_capturing,
        "has_frame": bool(rgb_bytes),
        "error": robot._last_capture_error,
        "save_path": robot._last_save_path,
    }


@app.get("/api/camera")
async def api_camera():
    """Return the latest captured frame (RGB + depth) as base64."""
    import base64
    rgb_bytes, depth_bytes = robot.get_latest_camera_bytes()
    if not rgb_bytes:
        return {"error": "no camera data — click 'capture' first"}
    return {
        "rgb_base64": base64.b64encode(rgb_bytes).decode('ascii'),
        "depth_base64": base64.b64encode(depth_bytes).decode('ascii'),
        "width": cfg.CAM_WIDTH, "height": cfg.CAM_HEIGHT,
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=cfg.HOST, port=cfg.PORT, reload=False)
