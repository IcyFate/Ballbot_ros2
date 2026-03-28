#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # publisher
        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)

        # jawna inicjalizacja I2C (ważne)
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # inicjalizacja IMU
        self.imu = LSM6DSO32(self.i2c, address=0x6A)

        self.get_logger().info("IMU initialized")

        # timer 50 Hz
        self.timer = self.create_timer(0.02, self.loop)

    def loop(self):
        try:
            acc = self.imu.acceleration
            gyro = self.imu.gyro

            msg = Imu()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # brak orientacji
            msg.orientation_covariance[0] = -1.0

            # akcelerometr (m/s^2)
            msg.linear_acceleration.x = acc[0]
            msg.linear_acceleration.y = acc[1]
            msg.linear_acceleration.z = acc[2]

            # żyroskop (rad/s)
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"I2C read error: {e}")


def main():
    rclpy.init()
    node = ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()