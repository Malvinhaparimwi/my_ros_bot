/**
 * useRos.js
 * Manages ROS connection state and auto-connects when the app mounts.
 * Also auto-starts robot.service on first successful connection.
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import RosService from '../services/RosService';
import { startRobotService, stopRobotService } from '../services/RobotSystemService';

export function useRos() {
  const [status, setStatus] = useState('disconnected'); // 'disconnected' | 'connecting' | 'connected' | 'reconnecting'
  const [robotServiceRunning, setRobotServiceRunning] = useState(false);
  const autoStartedRef = useRef(false);

  useEffect(() => {
    const onConnected = async () => {
      setStatus('connected');
      // Auto-start robot.service on first connection
      if (!autoStartedRef.current) {
        autoStartedRef.current = true;
        try {
          await startRobotService();
          setRobotServiceRunning(true);
        } catch (e) {
          console.warn('Could not auto-start robot.service:', e.message);
        }
      }
    };
    const onDisconnected = () => {
      setStatus('disconnected');
      autoStartedRef.current = false;
    };
    const onReconnecting = () => setStatus('reconnecting');

    RosService.on('connected', onConnected);
    RosService.on('disconnected', onDisconnected);
    RosService.on('reconnecting', onReconnecting);

    setStatus('connecting');
    RosService.connect();

    return () => {
      RosService.off('connected', onConnected);
      RosService.off('disconnected', onDisconnected);
      RosService.off('reconnecting', onReconnecting);
    };
  }, []);

  const startService = useCallback(async () => {
    try {
      await startRobotService();
      setRobotServiceRunning(true);
      return true;
    } catch (e) {
      console.error('startRobotService failed:', e);
      return false;
    }
  }, []);

  const stopService = useCallback(async () => {
    try {
      await stopRobotService();
      setRobotServiceRunning(false);
      return true;
    } catch (e) {
      console.error('stopRobotService failed:', e);
      return false;
    }
  }, []);

  return { status, robotServiceRunning, startService, stopService };
}
