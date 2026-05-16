/**
 * DPad.js
 * Touch-based directional pad — light theme.
 */

import React from 'react';
import { View, Text, StyleSheet, TouchableOpacity } from 'react-native';

const BTN_SIZE = 74;
const ICON = { forward: '▲', backward: '▼', left: '◀', right: '▶' };

function DPadButton({ direction, activeDir, onPressIn, onPressOut }) {
  const isActive = activeDir === direction;
  return (
    <TouchableOpacity
      style={[styles.btn, isActive && styles.btnActive]}
      onPressIn={() => onPressIn(direction)}
      onPressOut={onPressOut}
      delayPressIn={0}
      activeOpacity={0.7}
    >
      <Text style={[styles.icon, isActive && styles.iconActive]}>
        {ICON[direction]}
      </Text>
    </TouchableOpacity>
  );
}

export default function DPad({ activeDir, onPressIn, onPressOut, onStop }) {
  return (
    <View style={styles.wrapper}>
      {/* Row 1: Forward */}
      <View style={styles.row}>
        <View style={styles.spacer} />
        <DPadButton
          direction="forward"
          activeDir={activeDir}
          onPressIn={onPressIn}
          onPressOut={onPressOut}
        />
        <View style={styles.spacer} />
      </View>

      {/* Row 2: Left | Stop | Right */}
      <View style={styles.row}>
        <DPadButton
          direction="left"
          activeDir={activeDir}
          onPressIn={onPressIn}
          onPressOut={onPressOut}
        />
        <TouchableOpacity
          style={styles.stopBtn}
          onPress={onStop}
          activeOpacity={0.7}
        >
          <View style={styles.stopDot} />
          <Text style={styles.stopLabel}>STOP</Text>
        </TouchableOpacity>
        <DPadButton
          direction="right"
          activeDir={activeDir}
          onPressIn={onPressIn}
          onPressOut={onPressOut}
        />
      </View>

      {/* Row 3: Backward */}
      <View style={styles.row}>
        <View style={styles.spacer} />
        <DPadButton
          direction="backward"
          activeDir={activeDir}
          onPressIn={onPressIn}
          onPressOut={onPressOut}
        />
        <View style={styles.spacer} />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  wrapper: {
    alignItems: 'center',
    gap: 7,
  },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 7,
  },
  spacer: {
    width: BTN_SIZE,
    height: BTN_SIZE,
  },
  btn: {
    width: BTN_SIZE,
    height: BTN_SIZE,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#CBD5E1',
    backgroundColor: '#FFFFFF',
    alignItems: 'center',
    justifyContent: 'center',
    shadowColor: '#0F172A',
    shadowOpacity: 0.08,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 3 },
    elevation: 2,
  },
  btnActive: {
    backgroundColor: '#14532D',
    borderColor: '#14532D',
    shadowColor: '#16A34A',
    shadowOpacity: 0.3,
    shadowRadius: 12,
    shadowOffset: { width: 0, height: 4 },
    elevation: 6,
  },
  icon: {
    fontSize: 27,
    color: '#64748B',
  },
  iconActive: {
    color: '#FFFFFF',
  },
  stopBtn: {
    width: BTN_SIZE,
    height: BTN_SIZE,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#FECACA',
    backgroundColor: '#FFF1F2',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 4,
    shadowColor: '#EF4444',
    shadowOpacity: 0.12,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 3 },
    elevation: 2,
  },
  stopDot: {
    width: 13,
    height: 13,
    borderRadius: 4,
    backgroundColor: '#EF4444',
  },
  stopLabel: {
    fontSize: 9,
    letterSpacing: 1.2,
    color: '#DC2626',
    fontWeight: '800',
  },
});
