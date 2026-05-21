const host = window.location.hostname || '192.168.50.1';
const apiBase = `${window.location.protocol}//${host}:5001`;
const streamUrl = `ws://${host}:5001/stream`;
const rosUrl = `ws://${host}:9090`;

const els = {
  apiStatus: document.querySelector('#apiStatus'),
  rosStatus: document.querySelector('#rosStatus'),
  camStatus: document.querySelector('#camStatus'),
  image: document.querySelector('#cameraImage'),
  empty: document.querySelector('#cameraEmpty'),
  frameMeta: document.querySelector('#frameMeta'),
  serviceLog: document.querySelector('#serviceLog'),
  drawerServiceLog: document.querySelector('#drawerServiceLog'),
  pumpState: document.querySelector('#pumpState'),
  pumpToggle: document.querySelector('#pumpToggle'),
  activeCommand: document.querySelector('#activeCommand'),
  headingValue: document.querySelector('#headingValue'),
  drawer: document.querySelector('#drawer'),
  drawerBackdrop: document.querySelector('#drawerBackdrop'),
};

const twistCommands = {
  forward: twist(0, -0.8),
  backward: twist(0, 0.8),
  left: twist(-0.22, 0),
  right: twist(0.22, 0),
  stop: twist(0, 0),
};

let rosSocket = null;
let streamSocket = null;
let publishTimer = null;
let activeDir = null;
let advertised = new Set();
let subscriptions = new Map();
let lastImageUrl = null;
let fpsCount = 0;
let headingOffset = 0;
let lastRawHeading = null;
let pumpState = 'off';

function twist(linearX, angularZ) {
  return {
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  };
}

function setStatus(el, label, state) {
  el.textContent = label;
  el.classList.remove('good', 'bad');
  if (state === 'good') el.classList.add('good');
  if (state === 'bad') el.classList.add('bad');
}

async function pingApi() {
  try {
    const res = await fetch(`${apiBase}/ping`, { cache: 'no-store' });
    setStatus(els.apiStatus, res.ok ? 'API online' : 'API error', res.ok ? 'good' : 'bad');
  } catch (_) {
    setStatus(els.apiStatus, 'API offline', 'bad');
  }
}

function connectStream() {
  if (streamSocket) {
    streamSocket.onclose = null;
    streamSocket.close();
  }

  setStatus(els.camStatus, 'Camera...', null);
  streamSocket = new WebSocket(streamUrl);
  streamSocket.binaryType = 'arraybuffer';

  streamSocket.onopen = () => setStatus(els.camStatus, 'Camera live', 'good');
  streamSocket.onerror = () => setStatus(els.camStatus, 'Camera error', 'bad');
  streamSocket.onclose = () => {
    setStatus(els.camStatus, 'Camera off', 'bad');
    setTimeout(connectStream, 2000);
  };

  streamSocket.onmessage = event => {
    if (!(event.data instanceof ArrayBuffer)) return;
    const bytes = new Uint8Array(event.data);
    const tag = String.fromCharCode(bytes[0]);
    if (tag !== 'L') return;

    const blob = new Blob([event.data.slice(1)], { type: 'image/jpeg' });
    const url = URL.createObjectURL(blob);
    els.image.onload = () => {
      if (lastImageUrl) URL.revokeObjectURL(lastImageUrl);
      lastImageUrl = url;
    };
    els.image.src = url;
    els.empty.style.display = 'none';
    fpsCount += 1;
  };
}

function connectRos() {
  if (rosSocket) {
    rosSocket.onclose = null;
    rosSocket.close();
  }

  setStatus(els.rosStatus, 'ROS...', null);
  rosSocket = new WebSocket(rosUrl);

  rosSocket.onopen = () => {
    setStatus(els.rosStatus, 'ROS online', 'good');
    advertised.clear();
    advertise('/onyx/cmd_vel', 'geometry_msgs/Twist');
    advertise('/pump/controller', 'std_msgs/String');
    advertise('pump_control', 'std_msgs/String');
    subscribe('/pump/controller', 'std_msgs/String', msg => {
      pumpState = msg.data === 'on' ? 'on' : 'off';
      renderPump();
    });
    subscribe('/imu/data', 'sensor_msgs/Imu', updateHeading);
    subscribe('/imu', 'sensor_msgs/Imu', updateHeading);
  };

  rosSocket.onerror = () => setStatus(els.rosStatus, 'ROS error', 'bad');
  rosSocket.onclose = () => {
    setStatus(els.rosStatus, 'ROS offline', 'bad');
    stopDrive();
    setTimeout(connectRos, 2500);
  };

  rosSocket.onmessage = event => {
    try {
      const data = JSON.parse(event.data);
      if (data.op !== 'publish') return;
      const handlers = subscriptions.get(data.topic) || [];
      handlers.forEach(handler => handler(data.msg));
    } catch (_) {}
  };
}

function sendRos(payload) {
  if (rosSocket && rosSocket.readyState === WebSocket.OPEN) {
    rosSocket.send(JSON.stringify(payload));
    return true;
  }
  return false;
}

function advertise(topic, type) {
  if (advertised.has(topic)) return;
  advertised.add(topic);
  sendRos({ op: 'advertise', topic, type });
}

function publish(topic, msg) {
  sendRos({ op: 'publish', topic, msg });
}

function subscribe(topic, type, handler) {
  if (!subscriptions.has(topic)) {
    subscriptions.set(topic, []);
    sendRos({ op: 'subscribe', topic, type });
  }
  subscriptions.get(topic).push(handler);
}

function startDrive(dir) {
  if (activeDir === dir) return;
  stopDrive(false);
  activeDir = dir;
  els.activeCommand.textContent = dir.toUpperCase();
  document.querySelectorAll('.drive-button').forEach(button => {
    button.classList.toggle('active', button.dataset.dir === dir);
  });
  publish('/onyx/cmd_vel', twistCommands[dir]);
  publishTimer = setInterval(() => {
    publish('/onyx/cmd_vel', twistCommands[dir]);
  }, 100);
}

function stopDrive(sendStop = true) {
  clearInterval(publishTimer);
  publishTimer = null;
  activeDir = null;
  els.activeCommand.textContent = 'STOP';
  document.querySelectorAll('.drive-button').forEach(button => {
    button.classList.remove('active');
  });
  if (sendStop) publish('/onyx/cmd_vel', twistCommands.stop);
}

function renderPump() {
  const on = pumpState === 'on';
  els.pumpState.textContent = on ? 'ON' : 'OFF';
  els.pumpToggle.classList.toggle('on', on);
}

function setPump(state) {
  pumpState = state;
  renderPump();
  const msg = { data: state };
  publish('/pump/controller', msg);
  publish('pump_control', msg);
}

function quaternionToYawDeg(q) {
  const siny = 2 * (q.w * q.z + q.x * q.y);
  const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny, cosy) * 180 / Math.PI;
}

function updateHeading(msg) {
  lastRawHeading = quaternionToYawDeg(msg.orientation);
  const adjusted = lastRawHeading - headingOffset;
  const normalized = ((adjusted + 540) % 360) - 180;
  els.headingValue.textContent = `${normalized.toFixed(1)}°`;
}

async function service(action) {
  const message = `Running ${action}...`;
  if (els.serviceLog) els.serviceLog.textContent = message;
  els.drawerServiceLog.textContent = message;
  try {
    const res = await fetch(`${apiBase}/service/${action}`, { method: 'POST' });
    const data = await res.json();
    const output = JSON.stringify(data, null, 2);
    if (els.serviceLog) els.serviceLog.textContent = output;
    els.drawerServiceLog.textContent = output;
  } catch (error) {
    const output = String(error);
    if (els.serviceLog) els.serviceLog.textContent = output;
    els.drawerServiceLog.textContent = output;
  }
}

function openDrawer() {
  els.drawer.classList.add('open');
  els.drawer.setAttribute('aria-hidden', 'false');
  els.drawerBackdrop.hidden = false;
}

function closeDrawer() {
  els.drawer.classList.remove('open');
  els.drawer.setAttribute('aria-hidden', 'true');
  els.drawerBackdrop.hidden = true;
}

document.querySelector('#reconnectCamera').addEventListener('click', connectStream);
document.querySelector('#startService').addEventListener('click', () => service('start'));
document.querySelector('#restartService').addEventListener('click', () => service('restart'));
document.querySelector('#stopService').addEventListener('click', () => service('stop'));
document.querySelector('#statusService').addEventListener('click', () => service('status'));
document.querySelector('#drawerStartService').addEventListener('click', () => service('start'));
document.querySelector('#drawerRestartService').addEventListener('click', () => service('restart'));
document.querySelector('#drawerStopService').addEventListener('click', () => service('stop'));
document.querySelector('#drawerStatusService').addEventListener('click', () => service('status'));
document.querySelector('#menuOpen').addEventListener('click', openDrawer);
document.querySelector('#menuClose').addEventListener('click', closeDrawer);
document.querySelector('#drawerBackdrop').addEventListener('click', closeDrawer);
document.querySelector('#pumpToggle').addEventListener('click', () => {
  setPump(pumpState === 'on' ? 'off' : 'on');
});
document.querySelector('#stopDrive').addEventListener('click', () => stopDrive());
document.querySelector('#resetHeading').addEventListener('click', () => {
  if (lastRawHeading !== null) headingOffset = lastRawHeading;
  els.headingValue.textContent = '0.0°';
});

document.querySelectorAll('[data-dir]').forEach(button => {
  const dir = button.dataset.dir;
  button.addEventListener('pointerdown', event => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    startDrive(dir);
  });
  button.addEventListener('pointerup', () => stopDrive());
  button.addEventListener('pointercancel', () => stopDrive());
  button.addEventListener('pointerleave', () => {
    if (activeDir === dir) stopDrive();
  });
});

const keyMap = {
  w: 'forward',
  s: 'backward',
  a: 'left',
  d: 'right',
  ArrowUp: 'forward',
  ArrowDown: 'backward',
  ArrowLeft: 'left',
  ArrowRight: 'right',
};

window.addEventListener('keydown', event => {
  if (event.key === 'Escape') {
    closeDrawer();
    return;
  }
  if (event.repeat) return;
  if (event.code === 'Space') {
    event.preventDefault();
    stopDrive();
    return;
  }
  const dir = keyMap[event.key];
  if (dir) {
    event.preventDefault();
    startDrive(dir);
  }
});

window.addEventListener('keyup', event => {
  if (keyMap[event.key]) stopDrive();
});

window.addEventListener('beforeunload', () => {
  stopDrive();
  if (lastImageUrl) URL.revokeObjectURL(lastImageUrl);
});

setInterval(() => {
  els.frameMeta.textContent = `${fpsCount} fps`;
  fpsCount = 0;
}, 1000);

renderPump();
pingApi();
setInterval(pingApi, 3000);
connectRos();
connectStream();
