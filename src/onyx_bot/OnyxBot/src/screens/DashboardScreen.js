/**
 * DashboardScreen.js
 * Light theme dashboard with stats navigation.
 */

import React, { useState } from 'react';
import {
  View,
  Text,
  StyleSheet,
  SafeAreaView,
  StatusBar as RNStatusBar,
  TouchableOpacity,
  Modal,
  Pressable,
} from 'react-native';

import { useRos } from '../hooks/useRos';
import { useTeleop } from '../hooks/useTeleop';
import { usePump } from '../hooks/usePump';
import StatusBar from '../components/StatusBar';
import CameraView from '../components/CameraView';
import DPad from '../components/DPad';

export default function DashboardScreen() {
  const [statsVisible, setStatsVisible] = useState(false);
  const [autonomyActive, setAutonomyActive] = useState(false);
  const { status, robotServiceRunning, startService, stopService } = useRos();
  const connected = status === 'connected';

  const { activeDir, startDir, stopDir, sendStop } = useTeleop(connected);
  const { pumpState, setPump } = usePump(connected);

  const isOn = pumpState === 'on';

  return (
    <SafeAreaView style={styles.safe}>
      <RNStatusBar barStyle="dark-content" backgroundColor="#FFFFFF" />

      {/* ── Top status bar ─────────────────────────────── */}
      <StatusBar
        status={status}
        robotServiceRunning={robotServiceRunning}
        onStartService={startService}
        onStopService={stopService}
      />

      {/* ── Camera stream ──────────────────────────────── */}
      <CameraView connected={connected} />

      {/* ── Middle row: Stats card + Pump button ───────── */}
      <View style={styles.midRow}>
        {/* Stats navigation card */}
        <TouchableOpacity
          style={styles.statsCard}
          onPress={() => setStatsVisible(true)}
          activeOpacity={0.75}
        >
          {/* Top: icon + label */}
          <View style={styles.statsTop}>
            <View style={styles.statsIconBox}>
              <Text style={styles.statsIcon}>🌿</Text>
            </View>
            <Text style={styles.statsTitle}>FIELD STATS</Text>
          </View>

          {/* Bottom: description + arrow */}
          <View style={styles.statsBottom}>
            <Text style={styles.statsDesc}>Coverage · Health · Yield</Text>
            <View style={styles.statsArrowBox}>
              <Text style={styles.statsArrow}>→</Text>
            </View>
          </View>
        </TouchableOpacity>

        {/* ── Pump Push Button ─────────────────────────── */}
        <TouchableOpacity
          onPress={() => setPump(isOn ? 'off' : 'on')}
          disabled={!connected}
          activeOpacity={0.75}
          style={[
            styles.pumpOuter,
            isOn ? styles.pumpOuterOn : styles.pumpOuterOff,
            !connected && styles.pumpDisabled,
          ]}
        >
          <View
            style={[
              styles.pumpBody,
              isOn ? styles.pumpBodyOn : styles.pumpBodyOff,
            ]}
          >
            <View
              style={[
                styles.pumpLed,
                isOn ? styles.pumpLedOn : styles.pumpLedOff,
              ]}
            />
            <Text style={styles.pumpWordLabel}>SPRAY</Text>
            <Text
              style={[
                styles.pumpStateText,
                isOn ? styles.pumpStateOn : styles.pumpStateOff,
              ]}
            >
              {isOn ? 'ON' : 'OFF'}
            </Text>
          </View>
        </TouchableOpacity>
      </View>

      {/* ── D-Pad ────────────────────────────────────────── */}
      <View style={styles.drivePanel}>
        <Text style={styles.panelLabel}>DRIVE</Text>
        <DPad
          activeDir={activeDir}
          onPressIn={startDir}
          onPressOut={stopDir}
          onStop={sendStop}
        />
        <TouchableOpacity
          style={[
            styles.autonomyBtn,
            autonomyActive && styles.autonomyBtnActive,
            !connected && styles.autonomyBtnDisabled,
          ]}
          onPress={() => setAutonomyActive(active => !active)}
          disabled={!connected}
          activeOpacity={0.78}
        >
          <View
            style={[
              styles.autonomyDot,
              autonomyActive && styles.autonomyDotActive,
            ]}
          />
          <View style={styles.autonomyTextBlock}>
            <Text
              style={[
                styles.autonomyTitle,
                autonomyActive && styles.autonomyTitleActive,
              ]}
            >
              AUTONOMY
            </Text>
            <Text
              style={[
                styles.autonomyState,
                autonomyActive && styles.autonomyStateActive,
              ]}
            >
              {autonomyActive ? 'Active path spraying' : 'Tap to activate'}
            </Text>
          </View>
        </TouchableOpacity>
      </View>

      <Modal
        visible={statsVisible}
        transparent
        animationType="fade"
        onRequestClose={() => setStatsVisible(false)}
      >
        <Pressable
          style={styles.modalBackdrop}
          onPress={() => setStatsVisible(false)}
        >
          <Pressable style={styles.statsModal}>
            <View style={styles.modalHeader}>
              <View style={styles.modalTitleBlock}>
                <Text style={styles.modalIcon}>🌿</Text>
                <View>
                  <Text style={styles.modalEyebrow}>FIELD STATS</Text>
                  <Text style={styles.modalTitle}>Spraying Summary</Text>
                </View>
              </View>
              <TouchableOpacity
                style={styles.modalClose}
                onPress={() => setStatsVisible(false)}
                activeOpacity={0.75}
              >
                <Text style={styles.modalCloseText}>×</Text>
              </TouchableOpacity>
            </View>

            <View style={styles.statGrid}>
              <View style={[styles.statTile, styles.statTilePrimary]}>
                <Text style={styles.statLabel}>Field Coverage</Text>
                <Text style={[styles.statValue, styles.statValuePrimary]}>68%</Text>
                <View style={styles.progressTrack}>
                  <View style={styles.progressFill} />
                </View>
                <Text style={[styles.statMeta, styles.statMetaPrimary]}>
                  2.4 ha sprayed
                </Text>
              </View>
              <View style={styles.statTile}>
                <Text style={styles.statLabel}>Plant Health</Text>
                <Text style={styles.statValue}>Good</Text>
                <Text style={styles.statMeta}>Low stress seen</Text>
              </View>
              <View style={styles.statTile}>
                <Text style={styles.statLabel}>Yield Estimate</Text>
                <Text style={styles.statValue}>4.8 t/ha</Text>
                <Text style={styles.statMeta}>Projected harvest</Text>
              </View>
            </View>
          </Pressable>
        </Pressable>
      </Modal>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safe: {
    flex: 1,
    backgroundColor: '#F6F8F4',
  },

  // ── Middle row ───────────────────────────────────────
  midRow: {
    flexDirection: 'row',
    paddingHorizontal: 12,
    paddingVertical: 7,
    gap: 10,
    alignItems: 'center',
  },

  // ── Stats card ───────────────────────────────────────
  statsCard: {
    flex: 1,
    height: 58,
    backgroundColor: '#FFFFFF',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#BBF7D0',
    paddingHorizontal: 12,
    paddingVertical: 8,
    justifyContent: 'center',
    shadowColor: '#22C55E',
    shadowOpacity: 0.09,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 4 },
    elevation: 2,
  },
  statsTop: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  statsIconBox: {
    width: 34,
    height: 34,
    borderRadius: 10,
    backgroundColor: '#DCFCE7',
    alignItems: 'center',
    justifyContent: 'center',
  },
  statsIcon: {
    fontSize: 15,
  },
  statsTitle: {
    fontSize: 11,
    fontWeight: '800',
    letterSpacing: 1.1,
    color: '#14532D',
  },
  statsBottom: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginTop: 2,
    paddingLeft: 42,
  },
  statsDesc: {
    fontSize: 9,
    color: '#15803D',
    fontWeight: '600',
    letterSpacing: 0,
    flex: 1,
  },
  statsArrowBox: {
    width: 22,
    height: 22,
    borderRadius: 11,
    backgroundColor: '#16A34A',
    alignItems: 'center',
    justifyContent: 'center',
  },
  statsArrow: {
    fontSize: 13,
    color: '#FFFFFF',
    fontWeight: '700',
  },

  // ── Field stats modal ───────────────────────────────
  modalBackdrop: {
    flex: 1,
    backgroundColor: 'rgba(2, 6, 23, 0.64)',
    justifyContent: 'center',
    paddingHorizontal: 16,
  },
  statsModal: {
    backgroundColor: '#FFFFFF',
    borderRadius: 16,
    borderWidth: 2,
    borderColor: '#22C55E',
    overflow: 'hidden',
    shadowColor: '#22C55E',
    shadowOpacity: 0.32,
    shadowRadius: 24,
    shadowOffset: { width: 0, height: 12 },
    elevation: 12,
  },
  modalHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    backgroundColor: '#14532D',
    paddingHorizontal: 14,
    paddingVertical: 14,
  },
  modalTitleBlock: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
  },
  modalIcon: {
    width: 34,
    height: 34,
    borderRadius: 12,
    backgroundColor: '#DCFCE7',
    textAlign: 'center',
    textAlignVertical: 'center',
    fontSize: 18,
  },
  modalEyebrow: {
    fontSize: 9,
    fontWeight: '800',
    letterSpacing: 1.4,
    color: '#BBF7D0',
  },
  modalTitle: {
    marginTop: 2,
    fontSize: 17,
    fontWeight: '900',
    color: '#FFFFFF',
  },
  modalClose: {
    width: 34,
    height: 34,
    borderRadius: 12,
    backgroundColor: 'rgba(255,255,255,0.14)',
    alignItems: 'center',
    justifyContent: 'center',
  },
  modalCloseText: {
    fontSize: 22,
    lineHeight: 24,
    color: '#FFFFFF',
    fontWeight: '600',
  },
  statGrid: {
    gap: 9,
    padding: 14,
  },
  statTile: {
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#D1FAE5',
    backgroundColor: '#F7FDF9',
    paddingHorizontal: 12,
    paddingVertical: 11,
  },
  statTilePrimary: {
    backgroundColor: '#ECFDF5',
    borderColor: '#86EFAC',
    paddingVertical: 13,
  },
  statLabel: {
    fontSize: 11,
    fontWeight: '800',
    color: '#475569',
  },
  statValue: {
    marginTop: 3,
    fontSize: 22,
    fontWeight: '900',
    color: '#14532D',
  },
  statValuePrimary: {
    fontSize: 30,
  },
  progressTrack: {
    height: 6,
    borderRadius: 3,
    backgroundColor: '#BBF7D0',
    marginTop: 7,
    overflow: 'hidden',
  },
  progressFill: {
    width: '68%',
    height: '100%',
    borderRadius: 3,
    backgroundColor: '#16A34A',
  },
  statMeta: {
    marginTop: 2,
    fontSize: 10,
    fontWeight: '600',
    color: '#64748B',
  },
  statMetaPrimary: {
    marginTop: 7,
    color: '#15803D',
  },

  // ── Pump button ──────────────────────────────────────
  pumpOuter: {
    width: 98,
    height: 58,
    borderRadius: 12,
    borderWidth: 1,
    overflow: 'hidden',
  },
  pumpDisabled: {
    opacity: 0.48,
  },
  pumpOuterOff: {
    borderColor: '#FED7AA',
    shadowColor: '#FB923C',
    shadowOpacity: 0.12,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 3 },
    elevation: 2,
  },
  pumpOuterOn: {
    borderColor: '#A7F3D0',
    shadowColor: '#10B981',
    shadowOpacity: 0.16,
    shadowRadius: 9,
    shadowOffset: { width: 0, height: 3 },
    elevation: 3,
  },
  pumpBody: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 8,
    gap: 2,
  },
  pumpBodyOff: {
    backgroundColor: '#FFF7ED',
  },
  pumpBodyOn: {
    backgroundColor: '#ECFDF5',
  },
  pumpLed: {
    width: 7,
    height: 7,
    borderRadius: 4,
    marginBottom: 1,
  },
  pumpLedOff: {
    backgroundColor: '#F97316',
    shadowColor: '#F97316',
    shadowOpacity: 0.5,
    shadowRadius: 4,
    elevation: 2,
  },
  pumpLedOn: {
    backgroundColor: '#10B981',
    shadowColor: '#10B981',
    shadowOpacity: 0.55,
    shadowRadius: 6,
    elevation: 2,
  },
  pumpWordLabel: {
    fontSize: 8,
    letterSpacing: 1.5,
    color: '#64748B',
    fontWeight: '700',
  },
  pumpStateText: {
    fontSize: 15,
    fontWeight: '900',
    letterSpacing: 0,
  },
  pumpStateOff: {
    color: '#C2410C',
  },
  pumpStateOn: {
    color: '#047857',
  },
  pumpHint: {
    fontSize: 7,
    color: '#94A3B8',
    letterSpacing: 0,
    marginTop: 0,
  },

  // ── Drive panel ──────────────────────────────────────
  drivePanel: {
    flex: 1,
    marginHorizontal: 12,
    marginBottom: 10,
    borderWidth: 1,
    borderColor: '#E5E7EB',
    borderRadius: 12,
    padding: 10,
    backgroundColor: '#FFFFFF',
    alignItems: 'center',
    shadowColor: '#0F172A',
    shadowOpacity: 0.07,
    shadowRadius: 10,
    shadowOffset: { width: 0, height: 4 },
    elevation: 2,
  },
  panelLabel: {
    fontSize: 9,
    letterSpacing: 1.4,
    color: '#64748B',
    fontWeight: '800',
    marginBottom: 8,
  },
  autonomyBtn: {
    width: '100%',
    minHeight: 52,
    marginTop: 10,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#BBF7D0',
    backgroundColor: '#ECFDF5',
    paddingHorizontal: 14,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
    shadowColor: '#16A34A',
    shadowOpacity: 0.08,
    shadowRadius: 8,
    shadowOffset: { width: 0, height: 3 },
    elevation: 2,
  },
  autonomyBtnActive: {
    borderColor: '#14532D',
    backgroundColor: '#14532D',
    shadowOpacity: 0.22,
    shadowRadius: 12,
    elevation: 4,
  },
  autonomyBtnDisabled: {
    opacity: 0.46,
  },
  autonomyDot: {
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: '#22C55E',
  },
  autonomyDotActive: {
    backgroundColor: '#BBF7D0',
  },
  autonomyTextBlock: {
    flex: 1,
  },
  autonomyTitle: {
    fontSize: 12,
    fontWeight: '900',
    letterSpacing: 1,
    color: '#14532D',
  },
  autonomyTitleActive: {
    color: '#FFFFFF',
  },
  autonomyState: {
    marginTop: 1,
    fontSize: 10,
    fontWeight: '700',
    color: '#15803D',
  },
  autonomyStateActive: {
    color: '#DCFCE7',
  },
});
