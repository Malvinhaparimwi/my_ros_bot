/**
 * CameraView.js
 * Displays /camera/left/compressed (sensor_msgs/CompressedImage) via rosbridge.
 * Light theme.
 */

import React, { useState, useEffect, useRef } from 'react';
import {
  View,
  Image,
  Text,
  StyleSheet,
  Dimensions,
} from 'react-native';
import RosService from '../services/RosService';

const TOPIC = '/camera/left/compressed';
const MSG_TYPE = 'sensor_msgs/CompressedImage';
const { width: SCREEN_W } = Dimensions.get('window');
const CAM_W = SCREEN_W - 32;
const CAM_H = Math.round(CAM_W * (9 / 16));
const CORNER = 14;
const CORNER_T = 2;

export default function CameraView({ connected }) {
  const [imageUri, setImageUri] = useState(null);
  const [fps, setFps] = useState(0);
  const [paused, setPaused] = useState(false);
  const fpsCountRef = useRef(0);
  const pausedRef = useRef(false);
  const lastFpsUpdate = useRef(Date.now());

  useEffect(() => {
    if (!connected) return;

    const handler = msg => {
      if (pausedRef.current) return;
      const format = msg.format?.includes('png') ? 'png' : 'jpeg';
      setImageUri(`data:image/${format};base64,${msg.data}`);
      fpsCountRef.current += 1;
      const now = Date.now();
      if (now - lastFpsUpdate.current >= 1000) {
        setFps(fpsCountRef.current);
        fpsCountRef.current = 0;
        lastFpsUpdate.current = now;
      }
    };

    RosService.subscribe(TOPIC, MSG_TYPE, handler);
    return () => RosService.unsubscribe(TOPIC, handler);
  }, [connected]);

  return (
    <View style={styles.wrapper}>
      {/* Header */}
      <View style={styles.header}>
        <View style={styles.titleRow}>
          <View style={[styles.liveDot, connected && styles.liveDotActive]} />
          <Text style={styles.title}>Camera Feed</Text>
        </View>
        <View style={styles.fpsPill}>
          <Text style={styles.fps}>{connected ? `${fps} fps` : '-- fps'}</Text>
        </View>
        {/* <TouchableOpacity
          style={styles.pauseBtn}
          onPress={togglePause}
          activeOpacity={0.7}
        >
          <Text style={styles.pauseText}>
            {paused ? '▶  Resume' : '⏸  Freeze'}
          </Text>
        </TouchableOpacity> */}
      </View>

      {/* Frame */}
      <View style={[styles.frame, { width: CAM_W, height: CAM_H }]}>
        {!connected ? (
          <View style={styles.offline}>
            <Text style={styles.offlineIcon}>NO</Text>
            <Text style={styles.offlineText}>NO SIGNAL</Text>
            <Text style={styles.offlineSub}>Connect to robot hotspot</Text>
          </View>
        ) : !imageUri ? (
          <View style={styles.offline}>
            <Text style={styles.offlineText}>WAITING FOR STREAM</Text>
            {/* <Text style={styles.offlineSub}>Check camera topic</Text> */}
          </View>
        ) : (
          <Image
            source={{ uri: imageUri }}
            style={styles.image}
            resizeMode="contain"
          />
        )}

        {/* Corner brackets — blue on light */}
        <View style={styles.cornerTL} />
        <View style={styles.cornerTR} />
        <View style={styles.cornerBL} />
        <View style={styles.cornerBR} />

        {paused && (
          <View style={styles.pauseOverlay}>
            <Text style={styles.pauseOverlayText}>FROZEN</Text>
          </View>
        )}
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
  titleRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
  },
  liveDot: {
    width: 7,
    height: 7,
    borderRadius: 4,
    backgroundColor: '#CBD5E1',
  },
  liveDotActive: {
    backgroundColor: '#16A34A',
  },
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
  fps: {
    fontSize: 10,
    color: '#64748B',
    fontWeight: '700',
  },
  pauseBtn: {
    paddingHorizontal: 12,
    paddingVertical: 5,
    borderWidth: 1.5,
    borderColor: '#BFDBFE',
    borderRadius: 20,
    backgroundColor: '#EFF6FF',
  },
  pauseText: {
    fontSize: 10,
    color: '#1D4ED8',
    fontWeight: '600',
    letterSpacing: 0.5,
  },

  // Frame
  frame: {
    backgroundColor: '#111827', // keep dark so video pops
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
    width: '100%',
    height: '100%',
  },

  // No signal state
  offline: {
    alignItems: 'center',
    justifyContent: 'center',
    gap: 6,
  },
  offlineIcon: {
    fontSize: 18,
    color: '#1F2937',
    fontWeight: '900',
  },
  offlineText: {
    fontSize: 11,
    letterSpacing: 2.2,
    color: '#94A3B8',
    fontWeight: '800',
  },
  offlineSub: {
    fontSize: 10,
    color: '#CBD5E1',
    letterSpacing: 0,
  },

  // Corner brackets — blue accent
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

  // Freeze overlay
  pauseOverlay: {
    ...StyleSheet.absoluteFillObject,
    backgroundColor: 'rgba(15,23,42,0.55)',
    alignItems: 'center',
    justifyContent: 'center',
  },
  pauseOverlayText: {
    fontSize: 13,
    letterSpacing: 3,
    color: '#DCFCE7',
    fontWeight: '800',
  },
});
