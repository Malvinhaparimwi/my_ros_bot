/**
 * useImu.js
 * Subscribes to /imu (sensor_msgs/Imu) and computes heading from quaternion.
 * Exposes resetImu() which re-zeros the heading offset.
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import RosService from '../services/RosService';

function quaternionToYawDeg({ x, y, z, w }) {
  // yaw (rotation about Z) from quaternion
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp); // radians
  return (yaw * 180) / Math.PI; // degrees
}

export function useImu(connected) {
  const [heading, setHeading] = useState(0);
  const [rawImu, setRawImu] = useState(null);
  const offsetRef = useRef(0);

  useEffect(() => {
    if (!connected) return;

    const handler = (msg) => {
      setRawImu(msg);
      const q = msg.orientation;
      const yaw = quaternionToYawDeg(q);
      const adjusted = yaw - offsetRef.current;
      // Normalise to -180..180
      const normalised = ((adjusted + 540) % 360) - 180;
      setHeading(normalised);
    };

    RosService.subscribe('/imu', 'sensor_msgs/Imu', handler);
    return () => RosService.unsubscribe('/imu', handler);
  }, [connected]);

  const resetImu = useCallback(() => {
    if (rawImu) {
      const q = rawImu.orientation;
      offsetRef.current = quaternionToYawDeg(q);
      setHeading(0);
    }
  }, [rawImu]);

  return { heading, rawImu, resetImu };
}
