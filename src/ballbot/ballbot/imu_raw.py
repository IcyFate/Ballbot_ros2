#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import spidev
import time


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)

        # =========================
        # SPI init (hardware CS = GPIO8)
        # =========================
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # bus 0, CE0 (GPIO8)
        self.spi.max_speed_hz = 100000
        self.spi.mode = 0b00  # wymagane dla LSM6DSO32

        time.sleep(0.1)

        # =========================
        # Test komunikacji
        # =========================
        who = self.read_reg(0x0F)
        self.get_logger().info(f"WHO_AM_I = {hex(who)}")

        if who != 0x6C:
            self.get_logger().error("IMU not detected correctly!")

        # =========================
        # Init IMU
        # =========================
        self.init_imu()
        self.get_logger().info("IMU (SPI) initialized")

        # 200 Hz
        self.timer = self.create_timer(0.005, self.loop)
        self.last_time = None

    # =========================
    # SPI helpers (hardware CS)
    # =========================
    def write_reg(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value])

    def read_reg(self, reg):
        resp = self.spi.xfer2([reg | 0x80, 0x00])
        return resp[1]

    def read_bytes(self, reg, length):
        resp = self.spi.xfer2([reg | 0xC0] + [0x00] * length)
        return resp[1:]

    # =========================
    # IMU init
    # =========================
    def init_imu(self):
        # CTRL1_XL
        self.write_reg(0x10, 0b01001010)

        # CTRL2_G
        self.write_reg(0x11, 0b01000000)

        time.sleep(0.1)

    # =========================
    # conversion
    # =========================
    def to_int16(self, lo, hi):
        val = (hi << 8) | lo
        if val & 0x8000:
            val -= 65536
        return val

    def convert_acc(self, raw):
        return raw * 0.001196

    def convert_gyro(self, raw):
        return raw * 0.0001527

    # =========================
    # main loop
    # =========================
    def loop(self):
        try:
            now = self.get_clock().now()
            time_sec = now.nanoseconds * 1e-9

            if self.last_time is None:
                dt = 0.0
            else:
                dt = time_sec - self.last_time

            self.last_time = time_sec

            # ACC
            acc_bytes = self.read_bytes(0x28, 6)
            ax = self.to_int16(acc_bytes[0], acc_bytes[1])
            ay = self.to_int16(acc_bytes[2], acc_bytes[3])
            az = self.to_int16(acc_bytes[4], acc_bytes[5])

            # GYRO
            gyro_bytes = self.read_bytes(0x22, 6)
            gx = self.to_int16(gyro_bytes[0], gyro_bytes[1])
            gy = self.to_int16(gyro_bytes[2], gyro_bytes[3])
            gz = self.to_int16(gyro_bytes[4], gyro_bytes[5])

            # convert
            ax = self.convert_acc(ax)
            ay = self.convert_acc(ay)
            az = self.convert_acc(az)

            gx = self.convert_gyro(gx)
            gy = self.convert_gyro(gy)
            gz = self.convert_gyro(gz)

            # ROS msg
            msg = Imu()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'imu_link'

            msg.orientation_covariance[0] = -1.0

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            msg.angular_velocity_covariance[0] = dt

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"SPI read error: {e}")

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()


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