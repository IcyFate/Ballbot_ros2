import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
from adafruit_lsm6ds import Rate


class ImuKalmanSmoother:
    def __init__(
        self,
        process_var_gyro: float = 0.05,
        process_var_acc: float = 0.5,
        meas_var_gyro: float = 0.05,
        meas_var_acc: float = 0.5,
    ):
        self.x = np.zeros(6, dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 1000.0
        self.I = np.eye(6, dtype=np.float64)

        self.Q = np.diag(
            [process_var_gyro] * 3 +
            [process_var_acc] * 3
        ).astype(np.float64)

        self.R = np.diag(
            [meas_var_gyro] * 3 +
            [meas_var_acc] * 3
        ).astype(np.float64)

    def step(self, z: np.ndarray, dt: float):
        dt = max(float(dt), 1e-6)

        # predict: F = I
        self.P = self.P + self.Q * dt

        # innovation
        y = z - self.x

        # update: H = I
        S = self.P + self.R
        K = self.P @ np.linalg.pinv(S)

        self.x = self.x + K @ y
        self.P = (self.I - K) @ self.P

        return self.x.copy()


class ImuKalmanNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/imu/kalman_smoothed',
            10
        )

        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.imu = LSM6DSO32(self.i2c, address=0x6A)

        self.imu.accelerometer_data_rate = Rate.RATE_6_66K_HZ
        self.imu.gyro_data_rate = Rate.RATE_6_66K_HZ

        self.filter = ImuKalmanSmoother()

        self.last_time = self.get_clock().now()

        self.msg = Float64MultiArray()
        self.z = np.zeros(6, dtype=np.float64)

        self.timer = self.create_timer(1 / 1000, self.loop)

        self.get_logger().info("IMU + simplified Kalman initialized")

    def loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            dt = 0.001

        try:
            acc = self.imu.acceleration
            gyro = self.imu.gyro
        except OSError as e:
            self.get_logger().warning(f"I2C read error: {e}")
            return

        self.z[0] = gyro[0]
        self.z[1] = gyro[1]
        self.z[2] = gyro[2]
        self.z[3] = acc[0]
        self.z[4] = acc[1]
        self.z[5] = acc[2]

        filtered = self.filter.step(self.z, dt)

        stamp_sec = now.nanoseconds * 1e-9

        self.msg.data = [stamp_sec] + filtered.tolist()

        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = ImuKalmanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()