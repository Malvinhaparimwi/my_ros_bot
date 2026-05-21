# OnyxBot Companion Website

The robot API server now serves a browser dashboard at:

```text
http://192.168.50.1:5001/
```

It uses:

- `ws://192.168.50.1:5001/stream` for the left camera feed.
- `ws://192.168.50.1:9090` for rosbridge topic control.
- `POST /service/start|stop|restart|status` for robot stack service control.

Start the server on the Raspberry Pi:

```bash
python3 src/onyx_robot_server/ros_bot_server.py
```

Or run it through the existing `robot-api.service` if that is how the Pi is
configured.

The dashboard supports:

- Live left camera video.
- Start, stop, restart, and status for the robot stack.
- Pump toggle through `/pump/controller` and `pump_control`.
- Hold-to-drive controls through `/onyx/cmd_vel`.
- Keyboard drive controls: `W/S/A/D`, arrow keys, and space to stop.
- IMU heading from `/imu/data` or `/imu`.
