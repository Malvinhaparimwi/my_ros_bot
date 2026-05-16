/**
 * useTeleop.js
 * Publishes geometry_msgs/Twist to /onyx/cmd_vel.
 * Handles key-press-style hold: publishes on interval while a direction is held.
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import RosService from '../services/RosService';

const TOPIC = '/onyx/cmd_vel';
const MSG_TYPE = 'geometry_msgs/Twist';
const PUBLISH_RATE_MS = 100; // 10 Hz

const LINEAR_SPEED = 0.3;   // m/s
const ANGULAR_SPEED = 0.8;  // rad/s

const CMD = {
  forward:  { linear: { x: LINEAR_SPEED,  y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
  backward: { linear: { x: -LINEAR_SPEED, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
  left:     { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: ANGULAR_SPEED } },
  right:    { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -ANGULAR_SPEED } },
  stop:     { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
};

export function useTeleop(connected) {
  const [activeDir, setActiveDir] = useState(null);
  const intervalRef = useRef(null);
  const advertisedRef = useRef(false);

  useEffect(() => {
    if (connected && !advertisedRef.current) {
      RosService.advertise(TOPIC, MSG_TYPE);
      advertisedRef.current = true;
    }
    if (!connected) {
      advertisedRef.current = false;
    }
  }, [connected]);

  const startDir = useCallback((dir) => {
    if (!connected) return;
    setActiveDir(dir);
    // Immediate publish
    RosService.publish(TOPIC, CMD[dir]);
    // Then repeat at PUBLISH_RATE_MS
    clearInterval(intervalRef.current);
    intervalRef.current = setInterval(() => {
      RosService.publish(TOPIC, CMD[dir]);
    }, PUBLISH_RATE_MS);
  }, [connected]);

  const stopDir = useCallback(() => {
    clearInterval(intervalRef.current);
    setActiveDir(null);
    if (connected) {
      RosService.publish(TOPIC, CMD.stop);
    }
  }, [connected]);

  const sendStop = useCallback(() => {
    clearInterval(intervalRef.current);
    setActiveDir(null);
    if (connected) {
      RosService.publish(TOPIC, CMD.stop);
    }
  }, [connected]);

  // Cleanup on unmount
  useEffect(() => () => clearInterval(intervalRef.current), []);

  return { activeDir, startDir, stopDir, sendStop };
}
