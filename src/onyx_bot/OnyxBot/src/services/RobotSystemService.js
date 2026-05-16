/**
 * RobotSystemService.js
 * Controls robot.service on the Pi via a lightweight HTTP endpoint.
 *
 * On the Pi, run a small Flask/Express server (see setup notes below)
 * that accepts POST requests to start/stop the robot.service.
 *
 * Pi-side setup (python, runs as a systemd service called robot-api.service):
 * ─────────────────────────────────────────────────────────────────
 * from flask import Flask, request, jsonify
 * import subprocess
 * app = Flask(__name__)
 *
 * @app.route('/service/<action>', methods=['POST'])
 * def service(action):
 *     if action not in ('start', 'stop', 'restart', 'status'):
 *         return jsonify({'error': 'invalid'}), 400
 *     result = subprocess.run(
 *         ['sudo', 'systemctl', action, 'robot.service'],
 *         capture_output=True, text=True
 *     )
 *     return jsonify({'stdout': result.stdout, 'returncode': result.returncode})
 *
 * if __name__ == '__main__':
 *     app.run(host='0.0.0.0', port=5001)
 * ─────────────────────────────────────────────────────────────────
 * Add to /etc/sudoers (visudo):
 *   pi ALL=(ALL) NOPASSWD: /bin/systemctl start robot.service, \
 *                          /bin/systemctl stop robot.service, \
 *                          /bin/systemctl restart robot.service, \
 *                          /bin/systemctl status robot.service
 */

const PI_API_BASE = 'http://192.168.50.1:5001';
const TIMEOUT_MS = 5000;

async function _fetchWithTimeout(url, options = {}) {
  const controller = new AbortController();
  const id = setTimeout(() => controller.abort(), TIMEOUT_MS);
  try {
    const res = await fetch(url, { ...options, signal: controller.signal });
    clearTimeout(id);
    return res;
  } catch (e) {
    clearTimeout(id);
    throw e;
  }
}

export async function startRobotService() {
  const res = await _fetchWithTimeout(`${PI_API_BASE}/service/start`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
  });
  return res.json();
}

export async function stopRobotService() {
  const res = await _fetchWithTimeout(`${PI_API_BASE}/service/stop`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
  });
  return res.json();
}

export async function restartRobotService() {
  const res = await _fetchWithTimeout(`${PI_API_BASE}/service/restart`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
  });
  return res.json();
}

export async function getRobotServiceStatus() {
  const res = await _fetchWithTimeout(`${PI_API_BASE}/service/status`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
  });
  return res.json();
}
