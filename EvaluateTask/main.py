from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import uvicorn
from api import router, start_background_tasks, sim, manager, controller
import asyncio
import os

app = FastAPI()
app.include_router(router)


# Define WebSocket endpoints directly on app (not in router)
@app.websocket("/ws/images")
async def ws_images(websocket: WebSocket):
    await manager.connect_image(websocket)
    try:
        # keep connection open; images are pushed from background task
        while True:
            await asyncio.sleep(3600)
    except WebSocketDisconnect:
        manager.disconnect(websocket)


@app.websocket("/ws/control")
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


@app.on_event("startup")
async def startup_event():
    app.state.bg_task = asyncio.create_task(start_background_tasks())


@app.on_event("shutdown")
async def shutdown_event():
    task = getattr(app.state, "bg_task", None)
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


@app.get("/")
async def root():
    """Serve the main index.html"""
    return FileResponse("index.html", media_type="text/html")


if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
