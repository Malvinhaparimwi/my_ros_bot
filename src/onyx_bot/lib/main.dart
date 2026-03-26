import 'package:flutter/material.dart';
import 'dart:convert';
import 'dart:ui';
import 'dart:async';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';

void main() {
  runApp(const OnyxApp());
}

class OnyxApp extends StatelessWidget {
  const OnyxApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF0A0E21),
        colorScheme: const ColorScheme.dark(
          primary: Color(0xFF0FB9B1),
          secondary: Color(0xFF45AAF2),
        ),
      ),
      home: const RobotDashboard(),
    );
  }
}

class RobotDashboard extends StatefulWidget {
  const RobotDashboard({super.key});

  @override
  State<RobotDashboard> createState() => _RobotDashboardState();
}

class _RobotDashboardState extends State<RobotDashboard> {
  // Your Server IP
  final String robotIP = "10.140.21.25";

  WebSocketChannel? _channel;
  bool isConnected = false;
  bool isConnecting = false;
  bool showCamera = true;
  String statusMessage = "OFFLINE";

  Key _mjpegKey = UniqueKey();
  Timer? _movementTimer;
  Timer? _reconnectTimer;

  @override
  void initState() {
    super.initState();
    _connectToWebSocket();

    // Auto-reconnect loop every 5 seconds if not connected
    _reconnectTimer = Timer.periodic(const Duration(seconds: 5), (timer) {
      if (!isConnected && !isConnecting) {
        _connectToWebSocket();
      }
    });
  }

  // --- CONNECTION LOGIC ---
  Future<void> _connectToWebSocket() async {
    if (isConnecting) return;

    setState(() {
      isConnecting = true;
      statusMessage = "CONNECTING...";
    });

    try {
      final channel = WebSocketChannel.connect(Uri.parse('ws://$robotIP:9090'));

      // Timeout if the server doesn't respond in 3 seconds
      await channel.ready.timeout(const Duration(seconds: 3));

      channel.stream.listen(
        (message) {
          if (!isConnected) setState(() => isConnected = true);
        },
        onDone: () => _handleDisconnect("DISCONNECTED"),
        onError: (error) => _handleDisconnect("WS ERROR"),
        cancelOnError: true,
      );

      if (mounted) {
        setState(() {
          _channel = channel;
          isConnected = true;
          isConnecting = false;
          statusMessage = "SYSTEM ONLINE";
          _mjpegKey = UniqueKey(); // Refresh camera once WS is solid
        });
      }
    } catch (e) {
      _handleDisconnect("UNREACHABLE");
    }
  }

  void _handleDisconnect(String reason) {
    _channel?.sink.close();
    if (mounted) {
      setState(() {
        isConnected = false;
        isConnecting = false;
        _channel = null;
        statusMessage = reason;
      });
    }
  }

  // --- REFRESH ACTION ---
  void _hardRefresh() async {
    setState(() {
      showCamera = false;
      isConnected = false;
      isConnecting = false;
    });

    await Future.delayed(const Duration(milliseconds: 300));

    setState(() {
      showCamera = true;
      _mjpegKey = UniqueKey();
    });

    _connectToWebSocket();
  }

  // --- ROBOT COMMANDS ---
  void sendTwist(double linear, double angular) {
    if (_channel == null || !isConnected) return;

    final message = {
      "op": "publish",
      "topic": "/turtle1/cmd_vel",
      "msg": {
        "linear": {"x": linear, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular},
      },
    };

    try {
      _channel!.sink.add(jsonEncode(message));
    } catch (e) {
      _handleDisconnect("SEND ERROR");
    }
  }

  void _startMoving(double linear, double angular) {
    _movementTimer?.cancel();
    if (!isConnected) return;

    sendTwist(linear, angular);
    _movementTimer = Timer.periodic(const Duration(milliseconds: 100), (_) {
      sendTwist(linear, angular);
    });
  }

  void _stopMoving() {
    _movementTimer?.cancel();
    sendTwist(0.0, 0.0);
  }

  @override
  void dispose() {
    _reconnectTimer?.cancel();
    _movementTimer?.cancel();
    _channel?.sink.close();
    super.dispose();
  }

  // --- UI BUILDING ---
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          // BACKGROUND: CAMERA STREAM
          Positioned(
            top: 110,
            left: 0,
            right: 0,
            bottom: 345,
            child: Container(
              color: Colors.black,
              child: showCamera
                  ? Mjpeg(
                      key: _mjpegKey,
                      stream: "http://$robotIP:2003/stream",
                      isLive: true,
                      fit: BoxFit.cover,
                      error: (context, error, stack) =>
                          _buildErrorOverlay("SIGNAL LOST"),
                    )
                  : const Center(
                      child: CircularProgressIndicator(
                        color: Color(0xFF0FB9B1),
                      ),
                    ),
            ),
          ),

          // TOP HUD: LOGO & STATUS
          Positioned(
            top: 50,
            left: 20,
            right: 20,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text(
                      "ONYX_BOT",
                      style: TextStyle(
                        fontSize: 22,
                        fontWeight: FontWeight.w900,
                        letterSpacing: 2,
                      ),
                    ),
                    Row(
                      children: [
                        Container(
                          width: 8,
                          height: 8,
                          decoration: BoxDecoration(
                            color: isConnected
                                ? Colors.greenAccent
                                : Colors.redAccent,
                            shape: BoxShape.circle,
                          ),
                        ),
                        const SizedBox(width: 8),
                        Text(
                          statusMessage,
                          style: TextStyle(
                            color: isConnected
                                ? Colors.greenAccent
                                : Colors.redAccent,
                            fontSize: 10,
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
                // MANUAL REFRESH BUTTON
                IconButton(
                  icon: const Icon(Icons.sync, color: Colors.cyanAccent),
                  onPressed: _hardRefresh,
                ),
              ],
            ),
          ),

          // BOTTOM PANEL: CONTROLS
          Align(
            alignment: Alignment.bottomCenter,
            child: Container(
              padding: const EdgeInsets.only(top: 20, bottom: 40),
              decoration: BoxDecoration(
                color: Colors.black.withOpacity(0.85),
                border: const Border(
                  top: BorderSide(color: Colors.white12, width: 0.5),
                ),
              ),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Opacity(
                    opacity: isConnected ? 1.0 : 0.3,
                    child: AbsorbPointer(
                      absorbing: !isConnected,
                      child: _buildControlPad(),
                    ),
                  ),
                  const SizedBox(height: 30),
                  _buildTelemetryRow(),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildControlPad() {
    return Column(
      children: [
        _controlBtn(Icons.arrow_upward, 1.0, 0.0),
        const SizedBox(height: 10),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _controlBtn(Icons.arrow_back, 0.0, 1.0),
            const SizedBox(width: 20),
            _controlBtn(Icons.stop, 0.0, 0.0, isRed: true),
            const SizedBox(width: 20),
            _controlBtn(Icons.arrow_forward, 0.0, -1.0),
          ],
        ),
        const SizedBox(height: 10),
        _controlBtn(Icons.arrow_downward, -1.0, 0.0),
      ],
    );
  }

  Widget _controlBtn(
    IconData icon,
    double lin,
    double ang, {
    bool isRed = false,
  }) {
    return GestureDetector(
      onTapDown: (_) =>
          (lin == 0 && ang == 0) ? sendTwist(0, 0) : _startMoving(lin, ang),
      onTapUp: (_) => _stopMoving(),
      onTapCancel: () => _stopMoving(),
      child: Container(
        padding: const EdgeInsets.all(20),
        decoration: BoxDecoration(
          color: isRed
              ? Colors.red.withOpacity(0.1)
              : Colors.white.withOpacity(0.05),
          borderRadius: BorderRadius.circular(20),
          border: Border.all(color: isRed ? Colors.redAccent : Colors.white24),
        ),
        child: Icon(
          icon,
          color: isRed ? Colors.redAccent : Colors.white,
          size: 28,
        ),
      ),
    );
  }

  Widget _buildTelemetryRow() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: [
        _telemetryItem("LIN", isConnected ? "READY" : "--"),
        _telemetryItem("ANG", isConnected ? "READY" : "--"),
        _telemetryItem("IP", robotIP),
      ],
    );
  }

  Widget _telemetryItem(String label, String value) {
    return Column(
      children: [
        Text(
          value,
          style: const TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 13,
            color: Color(0xFF0FB9B1),
          ),
        ),
        Text(label, style: const TextStyle(color: Colors.white38, fontSize: 8)),
      ],
    );
  }

  Widget _buildErrorOverlay(String msg) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          const Icon(Icons.videocam_off, color: Colors.white10, size: 60),
          const SizedBox(height: 10),
          Text(
            msg,
            style: const TextStyle(color: Colors.white24, fontSize: 10),
          ),
        ],
      ),
    );
  }
}
