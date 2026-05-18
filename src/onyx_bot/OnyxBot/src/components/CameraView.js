/**
 * CameraView.js
 * Single camera feed (Pi Cam) via WebSocket.
 * Minimal base64 work, dirty-flag rendering, no right camera.
 */

import React, { useState, useEffect, useRef } from 'react';
import { View, Image, Text, StyleSheet, Dimensions } from 'react-native';

const WS_URL = 'ws://192.168.50.1:5001/stream'; // <-- your Pi IP:port

const { width: SCREEN_W } = Dimensions.get('window');
const CAM_W = SCREEN_W - 32;
const CAM_H = Math.round(CAM_W * (9 / 16));
const CORNER = 14;
const CORNER_T = 2;
const DISPLAY_INTERVAL_MS = 50; // 20 fps display tick — matches camera output

// ─── Fast ArrayBuffer → base64 ───────────────────────────────────────────────
const B64_CHARS =
  'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';

function bufferToBase64(buffer) {
  const bytes = new Uint8Array(buffer);
  const len = bytes.length;
  let out = '';
  let i = 0;
  for (; i + 2 < len; i += 3) {
    const n = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];
    out +=
      B64_CHARS[(n >> 18) & 63] +
      B64_CHARS[(n >> 12) & 63] +
      B64_CHARS[(n >> 6) & 63] +
      B64_CHARS[n & 63];
  }
  if (i < len) {
    const a = bytes[i];
    const b = i + 1 < len ? bytes[i + 1] : 0;
    const n = (a << 16) | (b << 8);
    out +=
      B64_CHARS[(n >> 18) & 63] +
      B64_CHARS[(n >> 12) & 63] +
      (i + 1 < len ? B64_CHARS[(n >> 6) & 63] : '=') +
      '=';
  }
  return out;
}

// ────────────────────────────────────────────────────────────────────────────

export default function CameraView({ connected }) {
  const [frameUri, setFrameUri] = useState(null);
  const [fps, setFps] = useState(0);
  const [wsStatus, setWsStatus] = useState('disconnected');

  const latestUri = useRef(null);
  const dirty = useRef(false);
  const fpsCount = useRef(0);
  const wsRef = useRef(null);
  const displayTimer = useRef(null);
  const fpsTimer = useRef(null);
  const reconnectTimer = useRef(null);

  const connect = () => {
    if (wsRef.current) {
      wsRef.current.onclose = null;
      wsRef.current.close();
    }

    setWsStatus('connecting');
    const ws = new WebSocket(WS_URL);
    ws.binaryType = 'arraybuffer';
    wsRef.current = ws;

    ws.onopen = () => {
      setWsStatus('connected');
      clearTimeout(reconnectTimer.current);
    };

    ws.onmessage = event => {
      if (!(event.data instanceof ArrayBuffer)) return;
      const buf = event.data;
      const tag = String.fromCharCode(new Uint8Array(buf)[0]);

      // Only handle left / Pi camera frames
      if (tag !== 'L') return;

      const jpeg = buf.slice(1);
      const b64 = bufferToBase64(jpeg);
      latestUri.current = `data:image/jpeg;base64,${b64}`;
      dirty.current = true;
      fpsCount.current += 1;
    };

    ws.onerror = () => setWsStatus('disconnected');

    ws.onclose = () => {
      setWsStatus('disconnected');
      reconnectTimer.current = setTimeout(() => {
        if (connected) connect();
      }, 2000);
    };
  };

  useEffect(() => {
    if (!connected) {
      clearTimeout(reconnectTimer.current);
      clearInterval(displayTimer.current);
      clearInterval(fpsTimer.current);
      if (wsRef.current) {
        wsRef.current.onclose = null;
        wsRef.current.close();
        wsRef.current = null;
      }
      latestUri.current = null;
      dirty.current = false;
      setFrameUri(null);
      setFps(0);
      setWsStatus('disconnected');
      return;
    }

    connect();

    // Only push a new URI to state when a fresh frame has arrived
    displayTimer.current = setInterval(() => {
      if (!dirty.current) return;
      dirty.current = false;
      setFrameUri(latestUri.current);
    }, DISPLAY_INTERVAL_MS);

    // FPS counter updates once per second
    fpsTimer.current = setInterval(() => {
      setFps(fpsCount.current);
      fpsCount.current = 0;
    }, 1000);

    return () => {
      clearTimeout(reconnectTimer.current);
      clearInterval(displayTimer.current);
      clearInterval(fpsTimer.current);
      if (wsRef.current) {
        wsRef.current.onclose = null;
        wsRef.current.close();
        wsRef.current = null;
      }
      latestUri.current = null;
    };
  }, [connected]);

  const isLive = wsStatus === 'connected';

  const placeholderText = {
    disconnected: 'NO SIGNAL',
    connecting: 'CONNECTING…',
    connected: 'WAITING FOR STREAM',
  }[wsStatus];

  const placeholderSub = {
    disconnected: 'Connect to robot hotspot',
    connecting: 'Reaching camera server…',
    connected: 'Check camera topic',
  }[wsStatus];

  return (
    <View style={styles.wrapper}>
      {/* Header */}
      <View style={styles.header}>
        <View style={styles.titleRow}>
          <View
            style={[
              styles.liveDot,
              isLive && !!frameUri && styles.liveDotActive,
            ]}
          />
          <Text style={styles.title}>Camera Feed</Text>
        </View>
        <View style={styles.fpsPill}>
          <Text style={styles.fpsText}>{isLive ? `${fps} fps` : '-- fps'}</Text>
        </View>
      </View>

      {/* Frame */}
      <View style={[styles.frame, { width: CAM_W, height: CAM_H }]}>
        {isLive && frameUri ? (
          <Image
            source={{ uri: frameUri }}
            style={styles.image}
            resizeMode="contain"
            fadeDuration={0}
          />
        ) : (
          <View style={styles.offline}>
            <Text style={styles.offlineText}>{placeholderText}</Text>
            <Text style={styles.offlineSub}>{placeholderSub}</Text>
          </View>
        )}

        <View style={styles.cornerTL} />
        <View style={styles.cornerTR} />
        <View style={styles.cornerBL} />
        <View style={styles.cornerBR} />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  wrapper: {
    marginHorizontal: 12,
    marginBottom: 8,
    marginTop: 12,
    alignItems: 'center',
  },
  header: {
    width: CAM_W,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 7,
  },
  titleRow: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  liveDot: {
    width: 7,
    height: 7,
    borderRadius: 4,
    backgroundColor: '#CBD5E1',
  },
  liveDotActive: { backgroundColor: '#16A34A' },
  title: {
    fontSize: 10,
    fontWeight: '800',
    letterSpacing: 1.2,
    color: '#14532D',
  },
  fpsPill: {
    minWidth: 54,
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 10,
    backgroundColor: '#F1F5F9',
    alignItems: 'center',
  },
  fpsText: { fontSize: 10, color: '#64748B', fontWeight: '700' },
  frame: {
    backgroundColor: '#111827',
    borderWidth: 1,
    borderColor: '#D1FAE5',
    borderRadius: 12,
    overflow: 'hidden',
    alignItems: 'center',
    justifyContent: 'center',
    shadowColor: '#0F172A',
    shadowOpacity: 0.12,
    shadowRadius: 12,
    shadowOffset: { width: 0, height: 4 },
    elevation: 3,
  },
  image: {
    ...StyleSheet.absoluteFillObject,
    width: '100%',
    height: '100%',
  },
  offline: { alignItems: 'center', justifyContent: 'center', gap: 6 },
  offlineText: {
    fontSize: 11,
    letterSpacing: 2.2,
    color: '#94A3B8',
    fontWeight: '800',
  },
  offlineSub: { fontSize: 10, color: '#CBD5E1' },
  cornerTL: {
    position: 'absolute',
    top: 10,
    left: 10,
    width: CORNER,
    height: CORNER,
    borderTopWidth: CORNER_T,
    borderLeftWidth: CORNER_T,
    borderColor: '#22C55E',
  },
  cornerTR: {
    position: 'absolute',
    top: 10,
    right: 10,
    width: CORNER,
    height: CORNER,
    borderTopWidth: CORNER_T,
    borderRightWidth: CORNER_T,
    borderColor: '#22C55E',
  },
  cornerBL: {
    position: 'absolute',
    bottom: 10,
    left: 10,
    width: CORNER,
    height: CORNER,
    borderBottomWidth: CORNER_T,
    borderLeftWidth: CORNER_T,
    borderColor: '#22C55E',
  },
  cornerBR: {
    position: 'absolute',
    bottom: 10,
    right: 10,
    width: CORNER,
    height: CORNER,
    borderBottomWidth: CORNER_T,
    borderRightWidth: CORNER_T,
    borderColor: '#22C55E',
  },
});
