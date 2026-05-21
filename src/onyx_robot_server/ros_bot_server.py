#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import subprocess
import uvicorn
from pathlib import Path

app = FastAPI()
WEB_DIR = Path(__file__).resolve().parent / 'web'
ALLOWED = {'start', 'stop', 'restart', 'status'}
STACK_UNITS = ('robot.service', 'robot_camera.service')

# Shared state
latest_left_frame: bytes | None = None
latest_right_frame: bytes | None = None
connected_clients: set[WebSocket] = set()
clients_lock = asyncio.Lock()
loop: asyncio.AbstractEventLoop = None

class CameraRelayNode(Node):
    def __init__(self):
        super().__init__('robot_api_camera_relay')
        self.create_subscription(
            CompressedImage,
            '/camera/left/compressed',
            self.left_callback,
            10,
        )
        self.create_subscription(
            CompressedImage,
            '/camera/right/compressed',
            self.right_callback,
            10,
        )

    def left_callback(self, msg):
        global latest_left_frame
        latest_left_frame = bytes(msg.data)
        asyncio.run_coroutine_threadsafe(
            broadcast(b'L' + latest_left_frame), loop
        )

    def right_callback(self, msg):
        global latest_right_frame
        latest_right_frame = bytes(msg.data)
        asyncio.run_coroutine_threadsafe(
            broadcast(b'R' + latest_right_frame), loop
        )

async def broadcast(data: bytes):
    async with clients_lock:
        dead = set()
        for ws in connected_clients:
            try:
                await ws.send_bytes(data)
            except Exception:
                dead.add(ws)
        connected_clients.difference_update(dead)

def start_ros_relay():
    def spin():
        rclpy.init(args=None)
        node = CameraRelayNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    threading.Thread(target=spin, daemon=True).start()

def run_systemctl(action, unit):
    result = subprocess.run(
        ['sudo', 'systemctl', action, unit],
        capture_output=True, text=True, timeout=10,
    )
    return {
        'unit': unit,
        'returncode': result.returncode,
        'stdout': result.stdout.strip(),
        'stderr': result.stderr.strip(),
        'success': result.returncode == 0,
    }

@app.get('/ping')
async def ping():
    return {'pong': True}

@app.get('/')
async def index():
    return FileResponse(WEB_DIR / 'index.html')

@app.post('/service/{action}')
async def service(action: str):
    if action not in ALLOWED:
        return JSONResponse({'error': f'Invalid action: {action}'}, status_code=400)
    units = STACK_UNITS if action != 'stop' else tuple(reversed(STACK_UNITS))
    results = [run_systemctl(action, unit) for unit in units]
    return {
        'action': action,
        'results': results,
        'success': all(r['success'] for r in results),
    }

@app.websocket('/stream')
async def stream(websocket: WebSocket):
    await websocket.accept()
    async with clients_lock:
        connected_clients.add(websocket)

    # Send latest frames immediately on connect so client isn't blank
    try:
        if latest_left_frame:
            await websocket.send_bytes(b'L' + latest_left_frame)
        if latest_right_frame:
            await websocket.send_bytes(b'R' + latest_right_frame)
        while True:
            await websocket.receive_text()  # keep connection alive
    except WebSocketDisconnect:
        pass
    finally:
        async with clients_lock:
            connected_clients.discard(websocket)

@app.on_event('startup')
async def startup():
    global loop
    loop = asyncio.get_event_loop()
    start_ros_relay()

app.mount('/static', StaticFiles(directory=WEB_DIR), name='static')

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5001)
