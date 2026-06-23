#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
from adafruit_lsm6ds import Rate


BURST_START_REG = 0x22
BURST_LEN = 12
BURST_CMD = bytes([BURST_START_REG])
BURST_STRUCT = struct.Struct('<hhhhhh')

GYRO_LSB_TO_RAD_S = 0.00875 * math.pi / 180.0
ACC_LSB_TO_MS2 = 0.244e-3 * 9.80665

ROLL_OFFSET = 0.006
PITCH_OFFSET = -0.025


def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch


class ImuRawPublisher(Node):
    def __init__(self):
        super().__init__('imu_raw_publisher')

        self.raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.tilt_pub = self.create_publisher(Float64MultiArray, '/imu/tilt', 10)

        self.declare_parameter('i2c_address', 0x6A)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('raw_topic_period_hz', 208.0)
        self.declare_parameter('filtered_topic', '/imu/data')

        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.filtered_topic = str(self.get_parameter('filtered_topic').value)
        period_hz = float(self.get_parameter('raw_topic_period_hz').value)

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = LSM6DSO32(self.i2c, address=self.i2c_address)
        self.imu.accelerometer_data_rate = Rate.RATE_208_HZ
        self.imu.gyro_data_rate = Rate.RATE_208_HZ

        self.rx = bytearray(BURST_LEN)
        self.i2c_read = self.i2c.writeto_then_readfrom

        self.raw_msg = Imu()
        self.raw_msg.orientation_covariance[0] = -1.0

        self.tilt_msg = Float64MultiArray()
        self.tilt_msg.data = [0.0, 0.0]

        self.create_subscription(Imu, self.filtered_topic, self.filtered_callback, 10)

        self.timer = self.create_timer(1.0 / period_hz, self.read_and_publish_raw)

    def read_and_publish_raw(self):
        try:
            self.i2c_read(self.i2c_address, BURST_CMD, self.rx)
        except OSError:
            return

        gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = BURST_STRUCT.unpack(self.rx)

        ax = ax_raw * ACC_LSB_TO_MS2
        ay = ay_raw * ACC_LSB_TO_MS2
        az = az_raw * ACC_LSB_TO_MS2

        gx = gx_raw * GYRO_LSB_TO_RAD_S
        gy = gy_raw * GYRO_LSB_TO_RAD_S
        gz = gz_raw * GYRO_LSB_TO_RAD_S

        msg = self.raw_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        self.raw_pub.publish(msg)

    def filtered_callback(self, msg: Imu):
        q = msg.orientation
        roll, pitch = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)

        self.tilt_msg.data[0] = roll + ROLL_OFFSET
        self.tilt_msg.data[1] = pitch + PITCH_OFFSET
        self.tilt_pub.publish(self.tilt_msg)


def main():
    rclpy.init()
    node = ImuRawPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()