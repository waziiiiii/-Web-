from typing import Set
from fastapi import WebSocket

class WSManager:
    def __init__(self):
        self.image_clients: Set[WebSocket] = set()

    async def connect_image(self, websocket: WebSocket):
        await websocket.accept()
        self.image_clients.add(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.image_clients:
            self.image_clients.remove(websocket)

    async def broadcast_image(self, img_b64: str):
        data = {"image": img_b64}
        to_remove = []
        for ws in list(self.image_clients):
            try:
                await ws.send_json(data)
            except Exception:
                to_remove.append(ws)

        for ws in to_remove:
            self.disconnect(ws)
