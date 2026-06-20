#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

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

PI = math.pi
TWO_PI = 2.0 * math.pi

G = 9.80665
ACC_K = 0.005
ACC_FULL_TRUST_ERROR = 0.25
ACC_NO_TRUST_ERROR = 1.0


class TiltEkf:
    __slots__ = (
        'roll',
        'pitch',
        'bx',
        'by',
        'bz',
        'initialized',
    )

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0

        self.bx = 0.0
        self.by = 0.0
        self.bz = 0.0

        self.initialized = False

    def step(self, ax, ay, az, gx, gy, gz, dt):
        if not self.initialized:
            self.roll = math.atan2(ay, az)
            self.pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

            self.bx = 0.0
            self.by = 0.0
            self.bz = 0.0

            self.initialized = True

        bx = self.bx
        by = self.by
        bz = self.bz

        p = gx - bx
        q = gy - by
        r = gz - bz

        roll = self.roll
        pitch = self.pitch

        sphi = math.sin(roll)
        cphi = math.cos(roll)
        tth = math.tan(pitch)

        roll += dt * (p + sphi * tth * q + cphi * tth * r)
        pitch += dt * (cphi * q - sphi * r)

        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        acc_error = abs(acc_norm - G)

        if acc_error < ACC_NO_TRUST_ERROR:
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

            y0 = roll_acc - roll
            if y0 > PI:
                y0 -= TWO_PI
            elif y0 < -PI:
                y0 += TWO_PI

            y1 = pitch_acc - pitch

            if acc_error <= ACC_FULL_TRUST_ERROR:
                k_acc = ACC_K
            else:
                scale = (ACC_NO_TRUST_ERROR - acc_error) / (
                    ACC_NO_TRUST_ERROR - ACC_FULL_TRUST_ERROR
                )
                k_acc = ACC_K * scale

            roll += k_acc * y0
            pitch += k_acc * y1

        self.roll = roll
        self.pitch = pitch

        return roll, pitch, p, q, r, bx, by, bz


class ImuKalmanNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/imu/kalman_state',
            1
        )

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = LSM6DSO32(self.i2c, address=0x6A)

        self.imu.accelerometer_data_rate = Rate.RATE_208_HZ
        self.imu.gyro_data_rate = Rate.RATE_208_HZ

        self.filter = TiltEkf()

        self.last_time = time.perf_counter()
        self.rx = bytearray(BURST_LEN)

        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 8

        self.i2c_read = self.i2c.writeto_then_readfrom
        self.publish = self.publisher.publish

    def loop_once(self):
        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0:
            dt = 0.001

        try:
            self.i2c_read(
                0x6A,
                BURST_CMD,
                self.rx
            )
        except OSError:
            return

        gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = BURST_STRUCT.unpack(self.rx)

        ax = ax_raw * ACC_LSB_TO_MS2
        ay = ay_raw * ACC_LSB_TO_MS2
        az = az_raw * ACC_LSB_TO_MS2

        gx = gx_raw * GYRO_LSB_TO_RAD_S
        gy = gy_raw * GYRO_LSB_TO_RAD_S
        gz = gz_raw * GYRO_LSB_TO_RAD_S

        roll, pitch, p, q, r, bx, by, bz = self.filter.step(
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            dt
        )

        data = self.msg.data
        data[0] = roll + ROLL_OFFSET
        data[1] = pitch + PITCH_OFFSET
        data[2] = p
        data[3] = q
        data[4] = r
        data[5] = bx
        data[6] = by
        data[7] = bz

        self.publish(self.msg)


def main():
    rclpy.init()
    node = ImuKalmanNode()

    try:
        while rclpy.ok():
            node.loop_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()