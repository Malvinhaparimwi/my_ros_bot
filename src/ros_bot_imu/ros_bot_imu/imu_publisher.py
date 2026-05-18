#!/usr/bin/env python3

import math
import serial
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


# ── Kalman filter (per axis) ──────────────────────────────────────────────────
class KalmanAngle:
    def __init__(self):
        self.angle     = 0.0
        self.bias      = 0.0
        self.P         = [[0.0, 0.0], [0.0, 0.0]]
        self.Q_angle   = 0.001
        self.Q_bias    = 0.003
        self.R_measure = 0.03

    def update(self, accel_angle, gyro_rate, dt):
        self.angle += dt * (gyro_rate - self.bias)
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S  = self.P[0][0] + self.R_measure
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        y = accel_angle - self.angle
        self.angle += K0 * y
        self.bias  += K1 * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]
        self.P[0][0] -= K0 * P00_temp
        self.P[0][1] -= K0 * P01_temp
        self.P[1][0] -= K1 * P00_temp
        self.P[1][1] -= K1 * P01_temp

        return self.angle


# ── Quaternion helpers ────────────────────────────────────────────────────────
def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll*0.5),  math.sin(roll*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cy, sy = math.cos(yaw*0.5),   math.sin(yaw*0.5)
    return (
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    )

def quat_multiply(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_conjugate(q):
    w,x,y,z = q
    return (w,-x,-y,-z)

def quat_to_euler(q):
    w,x,y,z = q
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class ImuNanoNode(Node):

    # Tune this to just above your sensor's idle noise floor (degrees/s).
    # Hold the sensor still, observe raw gz values, set slightly above max seen.
    GYRO_DEADBAND_DEG = 0.5   # °/s

    # Number of samples to average for startup bias calibration.
    CALIB_SAMPLES = 200

    def __init__(self):
        super().__init__('imu_nano')

        self.declare_parameter('port',     '/dev/serial/by-id/usb-Arduino_Nano_33_BLE_5121ED79AD58E624-if00')
        self.declare_parameter('baud',     115200)
        self.declare_parameter('frame_id', 'imu_link')

        port          = self.get_parameter('port').value
        baud          = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.kf_roll  = KalmanAngle()
        self.kf_pitch = KalmanAngle()

        self.zero_quat = (1.0, 0.0, 0.0, 0.0)
        self.last_quat = (1.0, 0.0, 0.0, 0.0)
        self.last_time = None
        self.yaw       = 0.0
        self.yaw_zero  = 0.0
        self.lock      = threading.Lock()

        # Gyro Z bias (rad/s) — estimated during calibration
        self.gz_bias = 0.0

        self.imu_pub   = self.create_publisher(Imu,            '/imu/data',        10)
        self.euler_pub = self.create_publisher(Vector3Stamped,  '/imu/euler',       10)
        self.swept_pub = self.create_publisher(Float32,         '/imu/angle_swept', 10)

        self.srv = self.create_service(Trigger, '/reset_imu', self.reset_callback)

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Opened {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            raise

        # Calibrate gyro bias before starting the reader thread
        self._calibrate_gyro()

        threading.Thread(target=self._reader, daemon=True).start()

    # ── Gyro bias calibration ─────────────────────────────────────────────────
    def _calibrate_gyro(self):
        """
        Collect CALIB_SAMPLES readings while the sensor is still and average
        the raw gz values to estimate the gyro Z-axis bias.
        Hold the sensor motionless during the ~2 s this takes.
        """
        self.get_logger().info(
            f'Calibrating gyro Z bias ({self.CALIB_SAMPLES} samples) — hold sensor still...'
        )
        samples = []
        while len(samples) < self.CALIB_SAMPLES:
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            if not raw.startswith('D '):
                continue

            parts = raw.split()
            if len(parts) != 13:
                continue

            try:
                vals = [float(p) for p in parts[1:]]
            except ValueError:
                continue

            lgz = vals[5]
            mgz = vals[11]
            samples.append(math.radians((lgz + mgz) * 0.5))

        self.gz_bias = sum(samples) / len(samples)
        self.get_logger().info(
            f'Gyro Z bias estimated: {math.degrees(self.gz_bias):.4f} °/s'
        )

    # ── Reset ─────────────────────────────────────────────────────────────────
    def reset_callback(self, request, response):
        with self.lock:
            self.zero_quat      = quat_conjugate(self.last_quat)
            self.yaw_zero       = self.yaw
            self.kf_roll.angle  = 0.0
            self.kf_roll.bias   = 0.0
            self.kf_pitch.angle = 0.0
            self.kf_pitch.bias  = 0.0
        self.get_logger().info('IMU zero-point reset!')
        response.success = True
        response.message = 'Zero point set to current orientation.'
        return response

    # ── Serial reader ─────────────────────────────────────────────────────────
    def _reader(self):
        deadband_rad = math.radians(self.GYRO_DEADBAND_DEG)

        while rclpy.ok():
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            if not raw.startswith('D '):
                self.get_logger().info(f'[nano] {raw}')
                continue

            parts = raw.split()
            if len(parts) != 13:
                continue

            try:
                vals = [float(p) for p in parts[1:]]
            except ValueError:
                continue

            lax,lay,laz, lgx,lgy,lgz = vals[0:6]
            max_,may,maz, mgx,mgy,mgz = vals[6:12]

            ax = (lax + max_) * 0.5
            ay = (lay + may)  * 0.5
            az = (laz + maz)  * 0.5
            gx = math.radians((lgx + mgx) * 0.5)
            gy = math.radians((lgy + mgy) * 0.5)

            # ── Bias-corrected gz with deadband ───────────────────────────────
            gz_raw = math.radians((lgz + mgz) * 0.5)
            gz_corrected = gz_raw - self.gz_bias
            gz = gz_corrected if abs(gz_corrected) > deadband_rad else 0.0

            accel_roll  = math.atan2(ay, az)
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

            now = self.get_clock().now()
            if self.last_time is None:
                self.last_time = now
                continue
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            if dt <= 0.0 or dt > 0.5:
                continue

            roll  = self.kf_roll.update(accel_roll,  gx, dt)
            pitch = self.kf_pitch.update(accel_pitch, gy, dt)

            with self.lock:
                self.yaw += gz * dt
                yaw      = self.yaw
                yaw_zero = self.yaw_zero

            quat = euler_to_quat(roll, pitch, yaw)

            with self.lock:
                self.last_quat = quat
                zero = self.zero_quat

            rel_q = quat_multiply(zero, quat)
            rel_roll, rel_pitch, rel_yaw = quat_to_euler(rel_q)

            stamp = now.to_msg()
            self._publish(quat, rel_q, rel_roll, rel_pitch, rel_yaw,
                          yaw, yaw_zero, stamp)

    # ── Publish ───────────────────────────────────────────────────────────────
    def _publish(self, quat, rel_q, roll, pitch, yaw, raw_yaw, yaw_zero, stamp):
        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.w   = quat[0]
        imu_msg.orientation.x   = quat[1]
        imu_msg.orientation.y   = quat[2]
        imu_msg.orientation.z   = quat[3]
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

        euler_msg = Vector3Stamped()
        euler_msg.header.stamp    = stamp
        euler_msg.header.frame_id = self.frame_id
        euler_msg.vector.x = math.degrees(roll)
        euler_msg.vector.y = math.degrees(pitch)
        euler_msg.vector.z = math.degrees(yaw)
        self.euler_pub.publish(euler_msg)

        # Unbounded accumulated angle from reset point.
        # Positive = one direction, negative = other, can exceed ±360°.
        swept = math.degrees(raw_yaw - yaw_zero)
        swept_msg = Float32()
        swept_msg.data = float(swept)
        self.swept_pub.publish(swept_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNanoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
