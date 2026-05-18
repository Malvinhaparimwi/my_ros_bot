/**
 * CameraView.js
 * Displays /camera/left/compressed (sensor_msgs/CompressedImage) via rosbridge.
 * Fixed: timer-driven rendering like the Python node, no memory leak.
 */

import React, { useState, useEffect, useRef } from 'react';
import {
  View,
  Image,
  Text,
  StyleSheet,
  Dimensions,
  Platform,
  requireNativeComponent,
} from 'react-native';
import RosService from '../services/RosService';

const TOPIC = '/camera/left/compressed';
const MSG_TYPE = 'sensor_msgs/CompressedImage';
const MJPEG_STREAM_URL = 'http://192.168.50.1:5001/camera.mjpg';
const DISPLAY_INTERVAL_MS = 33; // ~30 fps, matches Python timer
const ROS_THROTTLE_MS = 80;
const { width: SCREEN_W } = Dimensions.get('window');
const CAM_W = SCREEN_W - 32;
const CAM_H = Math.round(CAM_W * (9 / 16));
const CORNER = 14;
const CORNER_T = 2;
const OnyxMjpegView =
  Platform.OS === 'android' ? requireNativeComponent('OnyxMjpegView') : null;

// ─── decode only once, store raw string, never accumulate ───────────────────

function bytesToBase64(bytes) {
  const chars =
    'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
  let output = '';
  let i = 0;
  for (; i + 2 < bytes.length; i += 3) {
    const n = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];
    output += chars[(n >> 18) & 63];
    output += chars[(n >> 12) & 63];
    output += chars[(n >> 6) & 63];
    output += chars[n & 63];
  }
  if (i < bytes.length) {
    const a = bytes[i];
    const b = i + 1 < bytes.length ? bytes[i + 1] : 0;
    const n = (a << 16) | (b << 8);
    output += chars[(n >> 18) & 63];
    output += chars[(n >> 12) & 63];
    output += i + 1 < bytes.length ? chars[(n >> 6) & 63] : '=';
    output += '=';
  }
  return output;
}

function decodeFrame(msg) {
  // Returns a data URI string, or null on failure
  try {
    const format = msg.format?.includes('png') ? 'png' : 'jpeg';
    let b64;
    if (typeof msg.data === 'string') {
      b64 = msg.data;
    } else if (Array.isArray(msg.data)) {
      b64 = bytesToBase64(msg.data);
    } else if (msg.data && Array.isArray(msg.data.data)) {
      b64 = bytesToBase64(msg.data.data);
    } else if (msg.data && typeof msg.data.data === 'string') {
      b64 = msg.data.data;
    } else {
      return null;
    }
    return `data:image/${format};base64,${b64}`;
  } catch {
    return null;
  }
}

// ────────────────────────────────────────────────────────────────────────────

export default function CameraView({ connected }) {
  const useNativeStream =
    connected && Platform.OS === 'android' && OnyxMjpegView;

  const [frameUri, setFrameUri] = useState(null);
  const [fps, setFps] = useState(0);
  const [hasEverReceived, setHasEverReceived] = useState(false);

  // Refs hold mutable state that must NOT trigger re-renders
  const latestUriRef = useRef(null); // newest decoded URI — overwritten each msg
  const dirtyRef = useRef(false); // true when latestUriRef has a frame not yet shown
  const fpsCountRef = useRef(0);
  const lastFpsTickRef = useRef(Date.now());
  const displayTimerRef = useRef(null);
  const fpsTimerRef = useRef(null);

  useEffect(() => {
    if (useNativeStream) {
      setFrameUri(null);
      setFps(0);
      setHasEverReceived(false);
      return;
    }

    if (!connected) {
      // Clean up everything on disconnect
      latestUriRef.current = null;
      dirtyRef.current = false;
      setFrameUri(null);
      setFps(0);
      setHasEverReceived(false);
      return;
    }

    // ── ROS callback: decode once, store in ref, mark dirty ─────────────────
    const handler = msg => {
      const uri = decodeFrame(msg);
      if (!uri) return;

      // Overwrite previous frame — old string becomes unreferenced immediately
      latestUriRef.current = uri;
      dirtyRef.current = true;
      fpsCountRef.current += 1;
      setHasEverReceived(true);
    };

    // ── Display timer (~30 fps) — mirrors Python's create_timer(1/30) ────────
    // Only calls setState when there is actually a new frame (dirtyRef).
    // This means React never re-renders at 30 fps when the camera is idle.
    displayTimerRef.current = setInterval(() => {
      if (!dirtyRef.current) return;
      dirtyRef.current = false;
      // Swap ref into state — React batches this with nothing else, one render
      setFrameUri(latestUriRef.current);
    }, DISPLAY_INTERVAL_MS);

    // ── FPS counter (1 s tick) ───────────────────────────────────────────────
    fpsTimerRef.current = setInterval(() => {
      setFps(fpsCountRef.current);
      fpsCountRef.current = 0;
    }, 1000);

    RosService.subscribe(TOPIC, MSG_TYPE, handler, {
      throttle_rate: ROS_THROTTLE_MS,
      queue_length: 1,
    });

    return () => {
      clearInterval(displayTimerRef.current);
      clearInterval(fpsTimerRef.current);
      displayTimerRef.current = null;
      fpsTimerRef.current = null;
      RosService.unsubscribe(TOPIC, handler);
      // Explicitly null out the large string so GC can collect it
      latestUriRef.current = null;
      dirtyRef.current = false;
    };
  }, [connected, useNativeStream]);

  return (
    <View style={styles.wrapper}>
      {/* Header */}
      <View style={styles.header}>
        <View style={styles.titleRow}>
          <View style={[styles.liveDot, connected && styles.liveDotActive]} />
          <Text style={styles.title}>Camera Feed</Text>
        </View>
        <View style={styles.fpsPill}>
          <Text style={styles.fps}>
            {useNativeStream ? 'native' : connected ? `${fps} fps` : '-- fps'}
          </Text>
        </View>
      </View>

      {/* Frame */}
      <View style={[styles.frame, { width: CAM_W, height: CAM_H }]}>
        {!connected ? (
          <View style={styles.offline}>
            <Text style={styles.offlineIcon}>NO</Text>
            <Text style={styles.offlineText}>NO SIGNAL</Text>
            <Text style={styles.offlineSub}>Connect to robot hotspot</Text>
          </View>
        ) : useNativeStream ? (
          <OnyxMjpegView sourceUrl={MJPEG_STREAM_URL} style={styles.image} />
        ) : !frameUri ? (
          <View style={styles.offline}>
            <Text style={styles.offlineText}>WAITING FOR STREAM</Text>
            <Text style={styles.offlineSub}>
              {hasEverReceived ? 'Frames decoding…' : 'Check camera topic'}
            </Text>
          </View>
        ) : (
          <Image
            source={{ uri: frameUri }}
            style={styles.image}
            resizeMode="contain"
            fadeDuration={0}
          />
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
    gap: 8,
  },
  titleRow: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  liveDot: { width: 7, height: 7, borderRadius: 4, backgroundColor: '#CBD5E1' },
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
  fps: { fontSize: 10, color: '#64748B', fontWeight: '700' },
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
  image: { ...StyleSheet.absoluteFillObject, width: '100%', height: '100%' },
  offline: { alignItems: 'center', justifyContent: 'center', gap: 6 },
  offlineIcon: { fontSize: 18, color: '#1F2937', fontWeight: '900' },
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
