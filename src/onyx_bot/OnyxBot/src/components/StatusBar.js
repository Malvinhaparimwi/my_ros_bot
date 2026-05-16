/**
 * StatusBar.js
 * Top bar — light theme
 */

import React, { useEffect, useRef } from 'react';
import {
  View,
  Text,
  StyleSheet,
  Animated,
  TouchableOpacity,
} from 'react-native';

const STATUS_COLOR = {
  connected: '#16A34A',
  connecting: '#D97706',
  reconnecting: '#EA580C',
  disconnected: '#DC2626',
};

const STATUS_LABEL = {
  connected: 'ONLINE',
  connecting: 'CONNECTING',
  reconnecting: 'RECONNECTING',
  disconnected: 'OFFLINE',
};

export default function StatusBar({
  status,
  robotServiceRunning,
  onStartService,
  onStopService,
}) {
  const blink = useRef(new Animated.Value(1)).current;

  useEffect(() => {
    let animation;

    if (status !== 'connected') {
      animation = Animated.loop(
        Animated.sequence([
          Animated.timing(blink, {
            toValue: 0.2,
            duration: 600,
            useNativeDriver: true,
          }),
          Animated.timing(blink, {
            toValue: 1,
            duration: 600,
            useNativeDriver: true,
          }),
        ]),
      );
      animation.start();
    } else {
      blink.stopAnimation();
      blink.setValue(1);
    }

    return () => animation?.stop();
  }, [blink, status]);

  const dotColor = STATUS_COLOR[status] || '#DC2626';

  return (
    <View style={styles.bar}>
      {/* Left: connection status */}
      <View style={styles.left}>
        <Animated.View
          style={[styles.dot, { backgroundColor: dotColor, opacity: blink }]}
        />
        <Text style={[styles.statusText, { color: dotColor }]}>
          {STATUS_LABEL[status] || 'OFFLINE'}
        </Text>
        {/* <Text style={styles.addr}>192.168.43.1</Text> */}
      </View>

      {/* Centre: branding */}
      <Text style={styles.brand}>ONYX Bot</Text>

      {/* Right: robot service toggle */}
      <View style={styles.right}>
        {/* <Text style={styles.svcLabel}>robot.svc</Text> */}
        <TouchableOpacity
          style={[
            styles.svcBtn,
            robotServiceRunning ? styles.svcRunning : styles.svcStopped,
          ]}
          onPress={robotServiceRunning ? onStopService : onStartService}
          disabled={status !== 'connected'}
          activeOpacity={0.8}
        >
          <Text
            style={[
              styles.svcBtnText,
              robotServiceRunning
                ? styles.svcRunningText
                : styles.svcStoppedText,
            ]}
          >
            {robotServiceRunning ? '■ STOP' : '▶ START'}
          </Text>
        </TouchableOpacity>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  bar: {
    flexDirection: 'row',
    alignItems: 'center',
    minHeight: 58,
    paddingHorizontal: 14,
    paddingVertical: 10,
    borderBottomWidth: 1,
    borderColor: '#E5E7EB',
    backgroundColor: '#FFFFFF',
  },
  left: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
  },
  dot: {
    width: 9,
    height: 9,
    borderRadius: 5,
  },
  statusText: {
    fontSize: 10,
    fontWeight: '800',
    letterSpacing: 1.2,
  },
  addr: {
    fontSize: 9,
    color: '#94A3B8',
    marginLeft: 2,
    letterSpacing: 1,
  },
  brand: {
    fontSize: 16,
    fontWeight: '900',
    letterSpacing: 0,
    color: '#14532D',
    textAlign: 'center',
  },
  right: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'flex-end',
    gap: 6,
  },
  svcLabel: {
    fontSize: 9,
    color: '#94A3B8',
    letterSpacing: 1,
  },
  svcBtn: {
    minWidth: 74,
    paddingHorizontal: 10,
    paddingVertical: 7,
    borderRadius: 12,
    borderWidth: 1,
    alignItems: 'center',
  },
  svcRunning: {
    borderColor: '#FECACA',
    backgroundColor: '#FFF1F2',
  },
  svcStopped: {
    borderColor: '#BBF7D0',
    backgroundColor: '#ECFDF5',
  },
  svcBtnText: {
    fontSize: 9,
    letterSpacing: 0.6,
    fontWeight: '800',
  },
  svcRunningText: { color: '#DC2626' },
  svcStoppedText: { color: '#16A34A' },
});
