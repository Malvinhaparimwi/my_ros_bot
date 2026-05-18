#!/usr/bin/env python3
"""
robot_api.py  –  Pi-side REST API for robot stack control
Run this as a separate systemd service (robot-api.service).

Install: pip3 install flask
Run once to test: python3 robot_api.py

Sudoers entry needed (visudo), replacing Malvin if your user differs:
  Malvin ALL=(ALL) NOPASSWD: /bin/systemctl start robot.service, \
                            /bin/systemctl stop robot.service, \
                            /bin/systemctl restart robot.service, \
                            /bin/systemctl status robot.service, \
                            /bin/systemctl start robot_camera.service, \
                            /bin/systemctl stop robot_camera.service, \
                            /bin/systemctl restart robot_camera.service, \
                            /bin/systemctl status robot_camera.service
"""

import threading

from flask import Flask, Response, jsonify
import subprocess

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

app = Flask(__name__)
ALLOWED = {'start', 'stop', 'restart', 'status'}
STACK_UNITS = ('robot.service', 'robot_camera.service')
latest_camera_frame = None
camera_condition = threading.Condition()


class CameraRelayNode(Node):
    def __init__(self):
        super().__init__('robot_api_camera_relay')
        self.create_subscription(
            CompressedImage,
            '/camera/left/compressed',
            self.camera_callback,
            10,
        )

    def camera_callback(self, msg):
        global latest_camera_frame

        with camera_condition:
            latest_camera_frame = bytes(msg.data)
            camera_condition.notify_all()


def start_camera_relay():
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

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()


def run_systemctl(action, unit):
    result = subprocess.run(
        ['sudo', 'systemctl', action, unit],
        capture_output=True,
        text=True,
        timeout=10,
    )
    return {
        'unit': unit,
        'returncode': result.returncode,
        'stdout': result.stdout.strip(),
        'stderr': result.stderr.strip(),
        'success': result.returncode == 0,
    }


@app.route('/service/<action>', methods=['POST'])
def service(action):
    if action not in ALLOWED:
        return jsonify({'error': f'Invalid action: {action}'}), 400

    units = STACK_UNITS if action != 'stop' else tuple(reversed(STACK_UNITS))
    results = [run_systemctl(action, unit) for unit in units]

    return jsonify({
        'action': action,
        'results': results,
        'success': all(result['success'] for result in results),
    })


@app.route('/ping', methods=['GET'])
def ping():
    return jsonify({'pong': True})


@app.route('/camera.mjpg', methods=['GET'])
def camera_mjpg():
    def stream():
        while True:
            with camera_condition:
                camera_condition.wait_for(
                    lambda: latest_camera_frame is not None,
                    timeout=2.0,
                )
                frame = latest_camera_frame

            if frame is None:
                continue

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Cache-Control: no-cache\r\n\r\n'
                + frame +
                b'\r\n'
            )

    return Response(
        stream(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )


if __name__ == '__main__':
    start_camera_relay()
    # Bind to hotspot interface IP
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
