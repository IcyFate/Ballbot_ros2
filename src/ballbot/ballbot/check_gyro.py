#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

import board
import busio


CTRL2_G_REG = 0x11


class GyroModeCheckerNode(Node):
    def __init__(self):
        super().__init__('gyro_mode_checker')

        self.i2c = busio.I2C(board.SCL, board.SDA)

        # odczyt jednego bajtu z CTRL2_G
        reg = bytearray(1)

        self.i2c.writeto_then_readfrom(
            0x6A,
            bytes([CTRL2_G_REG]),
            reg
        )

        ctrl2_g = reg[0]

        self.get_logger().info(f"CTRL2_G = 0x{ctrl2_g:02X}")

        self.decode_ctrl2_g(ctrl2_g)

        # zakończ node po wypisaniu
        rclpy.shutdown()

    def decode_ctrl2_g(self, value: int):
        # ODR: bity [7:4]
        odr = (value >> 4) & 0x0F

        # FS_G: bity [3:2]
        fs_g = (value >> 2) & 0x03

        # FS_125: bit [1]
        fs_125 = (value >> 1) & 0x01

        # dekodowanie ODR
        odr_map = {
            0x0: "Power-down",
            0x1: "12.5 Hz",
            0x2: "26 Hz",
            0x3: "52 Hz",
            0x4: "104 Hz",
            0x5: "208 Hz",
            0x6: "416 Hz",
            0x7: "833 Hz",
            0x8: "1.66 kHz",
            0x9: "3.33 kHz",
            0xA: "6.66 kHz",
        }

        odr_str = odr_map.get(odr, "Unknown")

        # dekodowanie zakresu
        if fs_125 == 1:
            range_dps = 125
            sensitivity_mdps = 4.375
        else:
            range_map = {
                0: (250, 8.75),
                1: (500, 17.5),
                2: (1000, 35.0),
                3: (2000, 70.0),
            }

            range_dps, sensitivity_mdps = range_map.get(
                fs_g,
                ("Unknown", "Unknown")
            )

        self.get_logger().info(f"ODR = {odr_str}")
        self.get_logger().info(f"Gyro range = ±{range_dps} dps")
        self.get_logger().info(
            f"Sensitivity = {sensitivity_mdps} mdps/LSB"
        )

        if isinstance(sensitivity_mdps, (int, float)):
            scale_rad_s = sensitivity_mdps * 1e-3 * np.pi / 180.0

            self.get_logger().info(
                f"Scale = {scale_rad_s:.10f} rad/s/LSB"
            )


def main():
    rclpy.init()

    try:
        GyroModeCheckerNode()
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()