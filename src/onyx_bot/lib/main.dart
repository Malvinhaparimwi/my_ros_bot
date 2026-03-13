import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    const appTitle = "Onyx-Bott";
    return MaterialApp(
      title: appTitle,
      home: Scaffold(body: const RobotController()),
    );
  }
}

class RobotController extends StatelessWidget {
  const RobotController({super.key});

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 2,
      child: Scaffold(
        appBar: AppBar(
          title: Center(
            child: Text(
              "Onyx Bot",
              style: TextStyle(fontSize: 28, fontWeight: FontWeight(650)),
            ),
          ),
          bottom: const TabBar(
            tabs: [
              Tab(text: "Control"),
              Tab(text: "Data"),
            ],
          ),
        ),
        body: const TabBarView(
          children: [
            Keys(),
            Center(child: Text("This is Tab 2")),
          ],
        ),
      ),
    );
  }
}

class Keys extends StatelessWidget {
  const Keys({super.key});

  static final channel = WebSocketChannel.connect(
    Uri.parse('ws://10.42.0.1:9090'), // change to your ROS machine IP
  );

  static bool advertised = false;
  void advertise() {
    if (!advertised) {
      final advertiseMsg = {
        "op": "advertise",
        "topic": "/cmd_vel",
        "type": "geometry_msgs/msg/Twist",
      };

      channel.sink.add(jsonEncode(advertiseMsg));
      advertised = true;
    }
  }

  void sendTwist(double linear, double angular) {
    final message = {
      "op": "publish",
      "topic": "/cmd_vel",
      "msg": {
        "linear": {"x": linear, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular},
      },
    };

    channel.sink.add(jsonEncode(message));
  }

  Widget controlButton(String text) {
    return ElevatedButton(
      onPressed: () {
        if (text == "↑") {
          sendTwist(1.0, 0.0);
        }

        if (text == "↓") {
          sendTwist(-1.0, 0.0);
        }

        if (text == "←") {
          sendTwist(0.0, 1.0);
        }

        if (text == "→") {
          sendTwist(0.0, -1.0);
        }

        if (text == "STOP") {
          sendTwist(0.0, 0.0);
        }
      },
      style: ElevatedButton.styleFrom(minimumSize: const Size(80, 80)),
      child: Text(text, style: const TextStyle(fontSize: 20)),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // UP
          controlButton("↑"),

          const SizedBox(height: 20),

          // LEFT STOP RIGHT
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              controlButton("←"),
              const SizedBox(width: 20),
              controlButton("STOP"),
              const SizedBox(width: 20),
              controlButton("→"),
            ],
          ),

          const SizedBox(height: 20),

          // DOWN
          controlButton("↓"),
        ],
      ),
    );
  }
}
