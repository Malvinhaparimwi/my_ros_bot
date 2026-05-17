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

from flask import Flask, jsonify
import subprocess

app = Flask(__name__)
ALLOWED = {'start', 'stop', 'restart', 'status'}
STACK_UNITS = ('robot.service', 'robot_camera.service')


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


if __name__ == '__main__':
    # Bind to hotspot interface IP
    app.run(host='0.0.0.0', port=5001, debug=False)
