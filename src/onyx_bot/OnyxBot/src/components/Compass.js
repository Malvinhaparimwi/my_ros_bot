/**
 * Compass.js
 * Digital heading display in degrees
 */

import React from 'react';
import { View, Text, StyleSheet, TouchableOpacity } from 'react-native';

export default function Compass({ heading = 0, onReset }) {
  const displayDeg = Math.round(((heading % 360) + 360) % 360);

  return (
    <View style={styles.wrapper}>
      <Text style={styles.label}>HEADING</Text>
      <View style={styles.display}>
        <Text style={styles.degrees}>{displayDeg}°</Text>
      </View>
      <TouchableOpacity style={styles.resetBtn} onPress={onReset}>
        <Text style={styles.resetText}>ZERO IMU</Text>
      </TouchableOpacity>
    </View>
  );
}

const styles = StyleSheet.create({
  wrapper: {
    alignItems: 'center',
    paddingVertical: 14,
    backgroundColor: '#FFFFFF',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#D1FAE5',
    marginBottom: 12,
    shadowColor: '#0F172A',
    shadowOpacity: 0.08,
    shadowRadius: 10,
    shadowOffset: { width: 0, height: 3 },
    elevation: 2,
  },
  label: {
    fontSize: 10,
    fontWeight: '800',
    letterSpacing: 1.4,
    color: '#15803D',
    marginBottom: 8,
  },
  display: {
    alignItems: 'center',
    gap: 8,
  },
  degrees: {
    fontSize: 44,
    fontWeight: '900',
    color: '#14532D',
  },
  resetBtn: {
    marginTop: 12,
    paddingHorizontal: 14,
    paddingVertical: 7,
    borderWidth: 1,
    borderColor: '#BBF7D0',
    borderRadius: 10,
    backgroundColor: '#ECFDF5',
  },
  resetText: {
    fontSize: 10,
    fontWeight: '800',
    letterSpacing: 1,
    color: '#047857',
  },
});
