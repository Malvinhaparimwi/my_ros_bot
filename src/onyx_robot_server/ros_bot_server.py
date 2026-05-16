#!/usr/bin/env python3
"""
robot_api.py  –  Pi-side REST API for robot.service control
Run this as a separate systemd service (robot-api.service).

Install: pip3 install flask
Run once to test: python3 robot_api.py

Sudoers entry needed (visudo):
  pi ALL=(ALL) NOPASSWD: /bin/systemctl start robot.service, \
                         /bin/systemctl stop robot.service, \
                         /bin/systemctl restart robot.service, \
                         /bin/systemctl status robot.service
"""

from flask import Flask, jsonify
import subprocess

app = Flask(__name__)
ALLOWED = {'start', 'stop', 'restart', 'status'}


@app.route('/service/<action>', methods=['POST'])
def service(action):
    if action not in ALLOWED:
        return jsonify({'error': f'Invalid action: {action}'}), 400

    result = subprocess.run(
        ['sudo', 'systemctl', action, 'robot.service'],
        capture_output=True,
        text=True,
        timeout=10,
    )
    return jsonify({
        'action': action,
        'returncode': result.returncode,
        'stdout': result.stdout.strip(),
        'stderr': result.stderr.strip(),
        'success': result.returncode == 0,
    })


@app.route('/ping', methods=['GET'])
def ping():
    return jsonify({'pong': True})


if __name__ == '__main__':
    # Bind to hotspot interface IP
    app.run(host='0.0.0.0', port=5001, debug=False)
