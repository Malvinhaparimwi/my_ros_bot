import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';

class ROSConnection {
  final channel = WebSocketChannel.connect(Uri.parse('ws://192.168.1.50:9090'));

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
}
