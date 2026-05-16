/**
 * usePump.js
 * Subscribes to /pump/controller and exposes pump state.
 * Also publishes 'on'/'off' string commands to /pump/controller.
 */

import { useState, useEffect, useCallback } from 'react';
import RosService from '../services/RosService';

const TOPIC = '/pump/controller';
const MSG_TYPE = 'std_msgs/String';

export function usePump(connected) {
  const [pumpState, setPumpState] = useState('off');

  useEffect(() => {
    if (!connected) return;

    RosService.advertise(TOPIC, MSG_TYPE);

    const handler = (msg) => {
      setPumpState(msg.data === 'on' ? 'on' : 'off');
    };

    RosService.subscribe(TOPIC, MSG_TYPE, handler);
    return () => RosService.unsubscribe(TOPIC, handler);
  }, [connected]);

  const setPump = useCallback((state) => {
    if (!connected) return;
    RosService.publish(TOPIC, { data: state });
    setPumpState(state);
  }, [connected]);

  return { pumpState, setPump };
}
