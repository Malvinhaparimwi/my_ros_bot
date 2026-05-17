/**
 * usePump.js
 * Subscribes to /pump/controller and exposes pump state.
 * Publishes 'on'/'off' string commands to the current and legacy pump topics.
 */

import { useState, useEffect, useCallback } from 'react';
import RosService from '../services/RosService';

const TOPIC = '/pump/controller';
const LEGACY_TOPIC = 'pump_control';
const MSG_TYPE = 'std_msgs/String';

export function usePump(connected) {
  const [pumpState, setPumpState] = useState('off');

  useEffect(() => {
    if (!connected) return;

    RosService.advertise(TOPIC, MSG_TYPE);
    RosService.advertise(LEGACY_TOPIC, MSG_TYPE);

    const handler = (msg) => {
      setPumpState(msg.data === 'on' ? 'on' : 'off');
    };

    RosService.subscribe(TOPIC, MSG_TYPE, handler);
    return () => RosService.unsubscribe(TOPIC, handler);
  }, [connected]);

  const setPump = useCallback((state) => {
    if (!connected) return;
    const msg = { data: state };
    RosService.publish(TOPIC, msg);
    RosService.publish(LEGACY_TOPIC, msg);
    setPumpState(state);
  }, [connected]);

  return { pumpState, setPump };
}
