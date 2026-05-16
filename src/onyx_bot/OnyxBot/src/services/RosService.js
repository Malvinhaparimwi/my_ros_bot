/**
 * RosService.js
 * Manages the rosbridge WebSocket connection and all topic pub/sub.
 * Assumes rosbridge_suite is running on the Pi at ws://192.168.43.1:9090
 */

import { EventEmitter } from 'eventemitter3';

const ROS_BRIDGE_URL = 'ws://10.42.0.1:9090';

class RosService extends EventEmitter {
  constructor() {
    super();
    this.ws = null;
    this.connected = false;
    this.reconnectTimer = null;
    this.msgId = 0;
    this.subscriptions = {};
    this.advertised = {};
  }

  // ─── Connection ──────────────────────────────────────────────
  connect() {
    if (this.ws) {
      this.ws.close();
    }
    this.ws = new WebSocket(ROS_BRIDGE_URL);

    this.ws.onopen = () => {
      this.connected = true;
      this.emit('connected');
      this._resubscribeAll();
      this._readvertiseAll();
      clearTimeout(this.reconnectTimer);
    };

    this.ws.onclose = () => {
      this.connected = false;
      this.emit('disconnected');
      this._scheduleReconnect();
    };

    this.ws.onerror = e => {
      this.emit('error', e.message);
    };

    this.ws.onmessage = e => {
      try {
        const msg = JSON.parse(e.data);
        if (msg.op === 'publish' && this.subscriptions[msg.topic]) {
          this.subscriptions[msg.topic].forEach(cb => cb(msg.msg));
        }
      } catch (_) {}
    };
  }

  disconnect() {
    clearTimeout(this.reconnectTimer);
    if (this.ws) {
      this.ws.onclose = null; // prevent auto-reconnect on manual disconnect
      this.ws.close();
      this.ws = null;
    }
    this.connected = false;
    this.emit('disconnected');
  }

  _scheduleReconnect() {
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => {
      this.emit('reconnecting');
      this.connect();
    }, 3000);
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  // ─── Subscribe ───────────────────────────────────────────────
  subscribe(topic, type, callback) {
    if (!this.subscriptions[topic]) {
      this.subscriptions[topic] = [];
    }
    this.subscriptions[topic].push(callback);
    this._send({ op: 'subscribe', topic, type });
  }

  unsubscribe(topic, callback) {
    if (!this.subscriptions[topic]) return;
    if (callback) {
      this.subscriptions[topic] = this.subscriptions[topic].filter(
        cb => cb !== callback,
      );
    } else {
      delete this.subscriptions[topic];
    }
    if (!this.subscriptions[topic] || this.subscriptions[topic].length === 0) {
      this._send({ op: 'unsubscribe', topic });
    }
  }

  _resubscribeAll() {
    // Re-send subscribe for all active topics after reconnect
    Object.keys(this.subscriptions).forEach(topic => {
      if (this.subscriptions[topic].length > 0) {
        this._send({ op: 'subscribe', topic });
      }
    });
  }

  // ─── Advertise & Publish ─────────────────────────────────────
  advertise(topic, type) {
    this.advertised[topic] = type;
    this._send({ op: 'advertise', topic, type });
  }

  publish(topic, msg) {
    this._send({ op: 'publish', topic, msg });
  }

  _readvertiseAll() {
    Object.entries(this.advertised).forEach(([topic, type]) => {
      this._send({ op: 'advertise', topic, type });
    });
  }

  unadvertise(topic) {
    delete this.advertised[topic];
    this._send({ op: 'unadvertise', topic });
  }

  // ─── Service Calls ───────────────────────────────────────────
  callService(service, serviceType, args = {}) {
    return new Promise((resolve, reject) => {
      const id = `srv_${++this.msgId}`;
      const handler = e => {
        try {
          const data = JSON.parse(e.data);
          if (data.op === 'service_response' && data.id === id) {
            this.ws.removeEventListener('message', handler);
            if (data.result) resolve(data.values);
            else reject(data.values);
          }
        } catch (_) {}
      };
      if (this.ws) this.ws.addEventListener('message', handler);
      this._send({ op: 'call_service', service, type: serviceType, args, id });
      setTimeout(() => {
        if (this.ws) this.ws.removeEventListener('message', handler);
        reject(new Error('Service call timed out'));
      }, 5000);
    });
  }
}

export default new RosService();
