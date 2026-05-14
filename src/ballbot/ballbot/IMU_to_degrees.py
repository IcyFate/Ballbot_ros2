#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
from adafruit_lsm6ds import Rate


BURST_START_REG = 0x22
BURST_LEN = 12

GYRO_LSB_TO_RAD_S = (0.00875 * math.pi / 180.0)
ACC_LSB_TO_MS2 = (0.244e-3 * 9.80665)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class TiltEkf:
    def __init__(self):

        self.roll = 0.0
        self.pitch = 0.0

        self.bx = 0.0
        self.by = 0.0
        self.bz = 0.0

        self.P = 100.0
        self.I = 1.0

        self.q_angle = 0.002

        self.R = 1.5e-6

        self.initialized = False

    def initialize_from_acc(self, acc):

        ax, ay, az = acc

        self.roll = math.atan2(ay, az)

        self.pitch = math.atan2(
            -ax,
            math.sqrt(ay * ay + az * az)
        )

        self.bx = 0.0
        self.by = 0.0
        self.bz = 0.0

        self.initialized = True

    def predict(self, gyro, dt):

        gx, gy, gz = gyro

        phi = self.roll
        theta = self.pitch

        p = gx - self.bx
        q = gy - self.by
        r = gz - self.bz

        sphi = math.sin(phi)
        cphi = math.cos(phi)
        tth = math.tan(theta)

        phi_dot = p + sphi * tth * q + cphi * tth * r
        theta_dot = cphi * q - sphi * r

        self.roll += dt * phi_dot
        self.pitch += dt * theta_dot

        self.P += (self.q_angle * dt)

    def update_from_acc(self, acc):

        ax, ay, az = acc

        roll_acc = math.atan2(ay, az)

        pitch_acc = math.atan2(
            -ax,
            math.sqrt(ay * ay + az * az)
        )

        y0 = wrap_angle(roll_acc - self.roll)
        y1 = pitch_acc - self.pitch

        k = 0.15

        self.roll += k * y0
        self.pitch += k * y1

    def step(self, acc, gyro, dt):

        if not self.initialized:
            self.initialize_from_acc(acc)

        self.predict(gyro, dt)
        self.update_from_acc(acc)

        gx, gy, gz = gyro

        p = gx - self.bx
        q = gy - self.by
        r = gz - self.bz

        return (
            self.roll,
            self.pitch,
            p,
            q,
            r,
            self.bx,
            self.by,
            self.bz
        )


class ImuKalmanNode(Node):

    def __init__(self):

        super().__init__('imu_kalman_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/imu/kalman_state',
            10
        )

        self.roll_pub = self.create_publisher(
            Float64,
            '/imu/roll',
            10
        )

        self.pitch_pub = self.create_publisher(
            Float64,
            '/imu/pitch',
            10
        )

        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.imu = LSM6DSO32(
            self.i2c,
            address=0x6A
        )

        self.imu.accelerometer_data_rate = Rate.RATE_208_HZ
        self.imu.gyro_data_rate = Rate.RATE_208_HZ

        self.filter = TiltEkf()

        self.last_time = self.get_clock().now()

        self.rx = bytearray(BURST_LEN)

        self.msg = Float64MultiArray()

        self.roll_msg = Float64()
        self.pitch_msg = Float64()

        self.timer = self.create_timer(
            0.001,
            self.loop
        )

    def read_burst(self):

        self.i2c.writeto_then_readfrom(
            0x6A,
            bytes([BURST_START_REG]),
            self.rx
        )

        gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = struct.unpack(
            '<hhhhhh',
            self.rx
        )

        return (
            (
                ax_raw * ACC_LSB_TO_MS2,
                ay_raw * ACC_LSB_TO_MS2,
                az_raw * ACC_LSB_TO_MS2,
            ),
            (
                gx_raw * GYRO_LSB_TO_RAD_S,
                gy_raw * GYRO_LSB_TO_RAD_S,
                gz_raw * GYRO_LSB_TO_RAD_S,
            )
        )

    def loop(self):

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds * 1e-9

        self.last_time = now

        if dt <= 0.0:
            dt = 0.001

        try:
            acc, gyro = self.read_burst()
        except OSError:
            return

        roll, pitch, p, q, r, bx, by, bz = self.filter.step(
            acc,
            gyro,
            dt
        )

        roll += 0.0511799388
        pitch -= 0.0167266463

        self.msg.data = [
            roll,
            pitch,
            p,
            q,
            r,
            bx,
            by,
            bz
        ]

        self.roll_msg.data = float(roll)
        self.pitch_msg.data = float(pitch)

        self.publisher.publish(self.msg)

        self.roll_pub.publish(self.roll_msg)
        self.pitch_pub.publish(self.pitch_msg)


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