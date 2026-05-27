#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FieldTaskExecutor(Node):
    def __init__(self):
        super().__init__("field_task_executor")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_topic", "/onyx/cmd_vel"),
                ("move_base_enable_topic", "/move_base/enable"),
                ("move_base_status_topic", "/move_base/status"),
                ("move_base_row_end_topic", "/move_base/row_end"),
                ("imu_euler_topic", "/imu/euler"),
                ("end_confirm_sec", 0.25),
                ("control_rate_hz", 10.0),
                ("forward_speed", 0.07),
                ("end_advance_distance_m", 0.75),
                ("turn_speed", 0.07),
                ("right_turn_yaw_delta_deg", -90.0),
                ("turn_tolerance_deg", 3.0),
                ("turn_timeout_sec", 8.0),
                ("debug_log_period_sec", 1.0),
            ],
        )

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.move_base_enable_topic = self.get_parameter("move_base_enable_topic").value
        self.move_base_status_topic = self.get_parameter("move_base_status_topic").value
        self.move_base_row_end_topic = self.get_parameter("move_base_row_end_topic").value
        self.imu_euler_topic = self.get_parameter("imu_euler_topic").value
        self.end_confirm_sec = float(self.get_parameter("end_confirm_sec").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.end_advance_distance_m = float(
            self.get_parameter("end_advance_distance_m").value
        )
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.right_turn_yaw_delta_deg = float(
            self.get_parameter("right_turn_yaw_delta_deg").value
        )
        self.turn_tolerance_deg = float(self.get_parameter("turn_tolerance_deg").value)
        self.turn_timeout_sec = float(self.get_parameter("turn_timeout_sec").value)
        self.debug_log_period_sec = float(self.get_parameter("debug_log_period_sec").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.enable_pub = self.create_publisher(Bool, self.move_base_enable_topic, 10)
        self.status_sub = self.create_subscription(
            String, self.move_base_status_topic, self.status_callback, 10
        )
        self.row_end_sub = self.create_subscription(
            Bool, self.move_base_row_end_topic, self.row_end_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Vector3Stamped, self.imu_euler_topic, self.imu_callback, 10
        )

        self.move_base_status = "unknown"
        self.last_logged_status = None
        self.seen_tracking = False
        self.seen_drive_command = False
        self.cmd_msg_count = 0
        self.status_msg_count = 0
        self.row_end_msg_count = 0
        self.latest_cmd_text = "none"
        self.lost_since = None
        self.stopped_since = None
        self.row_end_since = None
        self.state = "following"
        self.state_started = self.get_clock().now()
        self.latest_yaw_deg = None
        self.turn_start_yaw_deg = None
        self.last_debug_log_time = None

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self._set_move_base_enabled(True)
        self.get_logger().info(
            "Task ready: follow row, advance 0.75 m after row loss, then turn right 90 deg."
        )
        self.get_logger().info(
            f"Topics: cmd_vel={self.cmd_vel_topic}, "
            f"enable={self.move_base_enable_topic}, "
            f"status={self.move_base_status_topic}, "
            f"row_end={self.move_base_row_end_topic}, "
            f"imu={self.imu_euler_topic}"
        )

    def status_callback(self, msg: String):
        self.status_msg_count += 1
        self.move_base_status = msg.data
        now = self.get_clock().now()

        if msg.data != self.last_logged_status:
            self.get_logger().info(f"move_base status: {msg.data}")
            self.last_logged_status = msg.data

        if msg.data in ("tracking", "realigning"):
            self.seen_tracking = True

        if msg.data in ("lost_recent", "lost"):
            if self.lost_since is None:
                self.lost_since = now
        else:
            self.lost_since = None

    def imu_callback(self, msg: Vector3Stamped):
        self.latest_yaw_deg = float(msg.vector.z)

    def row_end_callback(self, msg: Bool):
        self.row_end_msg_count += 1
        if self.state != "following" or not msg.data:
            return
        if self.row_end_since is None:
            self.row_end_since = self.get_clock().now()
            self.get_logger().info("move_base row_end event received")

    def cmd_callback(self, msg: Twist):
        self.cmd_msg_count += 1
        self.latest_cmd_text = (
            f"lin=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.linear.z:.3f}) "
            f"ang=({msg.angular.x:.3f},{msg.angular.y:.3f},{msg.angular.z:.3f})"
        )
        if self.state != "following":
            return

        moving = (
            abs(msg.linear.x) > 1e-4
            or abs(msg.linear.y) > 1e-4
            or abs(msg.linear.z) > 1e-4
            or abs(msg.angular.x) > 1e-4
            or abs(msg.angular.y) > 1e-4
            or abs(msg.angular.z) > 1e-4
        )
        if moving:
            self.seen_drive_command = True
            self.stopped_since = None
        elif self.seen_drive_command and self.stopped_since is None:
            self.stopped_since = self.get_clock().now()

    def tick(self):
        self._log_state_periodically()
        if self.state == "following":
            self._tick_following()
        elif self.state == "advance_after_end":
            self._tick_advance_after_end()
        elif self.state == "turn_right":
            self._tick_turn_right()
        elif self.state == "done":
            self._set_move_base_enabled(False)
            self._publish_stop()

    def _tick_following(self):
        self._set_move_base_enabled(True)

        trigger_time = self.row_end_since or self.lost_since or self.stopped_since
        if trigger_time is None:
            return

        lost_age = (self.get_clock().now() - trigger_time).nanoseconds / 1e9
        if lost_age < self.end_confirm_sec:
            return

        if self.row_end_since is not None:
            reason = "row_end event"
        elif self.lost_since is not None:
            reason = "no detections"
        else:
            reason = "move_base stopped"
        self.get_logger().info(
            f"Move_base reports {reason} ({self.move_base_status}); "
            f"advancing {self.end_advance_distance_m:.2f} m toward row end."
        )
        self._set_move_base_enabled(False)
        self._publish_stop()
        self.lost_since = None
        self.stopped_since = None
        self.row_end_since = None
        self._start_state("advance_after_end")

    def _tick_advance_after_end(self):
        self._set_move_base_enabled(False)

        elapsed = self._state_elapsed_sec()
        distance_m = abs(self.forward_speed) * elapsed
        if distance_m >= self.end_advance_distance_m:
            self._publish_stop()
            self.turn_start_yaw_deg = self.latest_yaw_deg
            self._start_state("turn_right")
            if self.turn_start_yaw_deg is None:
                self.get_logger().warning(
                    "No IMU yaw received yet; turning with timeout fallback."
                )
            else:
                self.get_logger().info(
                    f"Advanced {distance_m:.2f} m; turning right from yaw "
                    f"{self.turn_start_yaw_deg:.1f} deg."
                )
            return

        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_pub.publish(self._to_microcontroller_twist(twist))

    def _tick_turn_right(self):
        self._set_move_base_enabled(False)

        elapsed = self._state_elapsed_sec()
        if self._right_turn_complete() or elapsed >= self.turn_timeout_sec:
            self._publish_stop()
            self.state = "done"
            self.get_logger().info("Right turn complete; task stopped.")
            return

        twist = Twist()
        twist.angular.z = -abs(self.turn_speed)
        self.cmd_pub.publish(self._to_microcontroller_twist(twist))

    def _right_turn_complete(self) -> bool:
        if self.turn_start_yaw_deg is None or self.latest_yaw_deg is None:
            return False

        yaw_delta = self._wrap_degrees(self.latest_yaw_deg - self.turn_start_yaw_deg)
        error = self._wrap_degrees(yaw_delta - self.right_turn_yaw_delta_deg)
        return abs(error) <= self.turn_tolerance_deg

    def _start_state(self, state: str):
        self.state = state
        self.state_started = self.get_clock().now()

    def _state_elapsed_sec(self) -> float:
        if self.state_started is None:
            self.state_started = self.get_clock().now()
            return 0.0
        return (self.get_clock().now() - self.state_started).nanoseconds / 1e9

    def _log_state_periodically(self):
        now = self.get_clock().now()
        if self.last_debug_log_time is not None:
            elapsed = (now - self.last_debug_log_time).nanoseconds / 1e9
            if elapsed < self.debug_log_period_sec:
                return

        self.last_debug_log_time = now
        state_age = self._state_elapsed_sec() if self.state_started is not None else 0.0
        row_end = self.row_end_since is not None
        lost = self.lost_since is not None
        stopped = self.stopped_since is not None
        yaw = "none" if self.latest_yaw_deg is None else f"{self.latest_yaw_deg:.1f}"
        cmd_pubs = self.count_publishers(self.cmd_vel_topic)
        cmd_subs = self.count_subscribers(self.cmd_vel_topic)
        status_pubs = self.count_publishers(self.move_base_status_topic)
        row_end_pubs = self.count_publishers(self.move_base_row_end_topic)
        self.get_logger().info(
            f"task_state={self.state} age={state_age:.1f}s "
            f"move_base_status={self.move_base_status} "
            f"row_end={row_end} lost={lost} stopped={stopped} "
            f"seen_tracking={self.seen_tracking} "
            f"seen_drive_command={self.seen_drive_command} yaw={yaw} "
            f"msgs(cmd={self.cmd_msg_count},status={self.status_msg_count},"
            f"row_end={self.row_end_msg_count}) "
            f"graph(cmd_pubs={cmd_pubs},cmd_subs={cmd_subs},"
            f"status_pubs={status_pubs},row_end_pubs={row_end_pubs}) "
            f"last_cmd={self.latest_cmd_text}"
        )

    def _set_move_base_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _to_microcontroller_twist(twist: Twist) -> Twist:
        converted = Twist()
        converted.linear.x = -twist.angular.z
        converted.angular.z = -twist.linear.x
        return converted

    @staticmethod
    def _wrap_degrees(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0


def main(args=None):
    rclpy.init(args=args)
    node = FieldTaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._set_move_base_enabled(False)
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
