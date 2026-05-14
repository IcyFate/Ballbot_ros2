#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# MODEL

r_k = 0.024

# LQR

K1 = -5.0
K2 = -1
K3 = 0
K4 = 0

# SILNIKI

MIN_COMMAND_RAD = 0

# GEOMETRIA

SQRT3_2 = 0.86602540378
SQRT2_2 = 0.70710678

MAX_ACC = 7
MAX_VEL = 1

ANGLE_DEADBAND = 0.005
RATE_DEADBAND = 0.01

# NODE

class LqrBalanceController(Node):

    def __init__(self):
        super().__init__('lqr_balance_controller')

        self.roll = 0.0
        self.pitch = 0.0

        self.roll_rate = 0.0
        self.pitch_rate = 0.0

        self.pos_x = 0.0
        self.pos_y = 0.0

        self.vel_x = 0.0
        self.vel_y = 0.0

        self.pos_x_meas = 0.0
        self.pos_y_meas = 0.0

        self.vel_x_meas = 0.0
        self.vel_y_meas = 0.0

        self.last_time = self.get_clock().now()

        self.log_counter = 0

        self.create_subscription(
            Float64MultiArray,
            '/imu/kalman_state',
            self.imu_callback,
            1
        )

        self.create_subscription(
            Float64MultiArray,
            'wheel_state',
            self.wheel_callback,
            1
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            'vel_from_controller',
            1
        )

        self.msg = Float64MultiArray()
        self.msg.data = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(
            0.004,
            self.control_loop
        )

    def imu_callback(self, msg):

        d = msg.data

        if len(d) < 4:
            return

        self.roll = d[0]
        self.pitch = d[1]

        self.roll_rate = d[2]
        self.pitch_rate = d[3]

    def wheel_callback(self, msg):

        d = msg.data

        if len(d) < 16:
            return

        # pozycje kół z enkoderów
        d1 = float(d[3])
        d2 = float(d[8])
        d3 = float(d[13])

        # prędkości liniowe kół z enkoderów
        v1 = float(d[5])
        v2 = float(d[10])
        v3 = float(d[15])

        c = SQRT2_2

        # pozycja platformy w układzie robota
        vy_r_pos = -d1 / c
        vx_r_pos = (d3 - d2) / (2.0 * SQRT3_2 * c)

        # pozycja platformy w układzie globalnym używanym w regulatorze
        self.pos_x_meas = c * vx_r_pos + c * vy_r_pos
        self.pos_y_meas = -c * vx_r_pos + c * vy_r_pos

        # prędkość platformy w układzie robota
        vy_r_vel = -v1 / c
        vx_r_vel = (v3 - v2) / (2.0 * SQRT3_2 * c)

        # prędkość platformy w układzie globalnym używanym w regulatorze
        self.vel_x_meas = c * vx_r_vel + c * vy_r_vel
        self.vel_y_meas = -c * vx_r_vel + c * vy_r_vel

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def deadband(self, x, threshold):

        if abs(x) < threshold:
            return 0.0

        return x

    def min_command_filter(self, x):

        if abs(x) < MIN_COMMAND_RAD:
            return 0.0

        return x

    def control_loop(self):

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds * 1e-9

        self.last_time = now

        if dt <= 0.0:
            return

        if dt > 0.02:
            dt = 0.02

        # FILTR MAŁYCH DRGAŃ

        theta_x = self.deadband(self.pitch, ANGLE_DEADBAND)
        theta_y = self.deadband(self.roll, ANGLE_DEADBAND)

        theta_dot_x = self.deadband(self.pitch_rate, RATE_DEADBAND)
        theta_dot_y = self.deadband(self.roll_rate, RATE_DEADBAND)

        # STAN Z ENKODERÓW

        self.pos_x = self.pos_x_meas
        self.pos_y = self.pos_y_meas

        self.vel_x = self.vel_x_meas
        self.vel_y = self.vel_y_meas

        # LQR

        ax = -(K1 * theta_x + K2 * theta_dot_x + K3 * self.pos_x + K4 * self.vel_x)
        ay = -(K1 * theta_y + K2 * theta_dot_y + K3 * self.pos_y + K4 * self.vel_y)

        # LIMIT ACC NA NORMĘ WEKTORA

        acc_norm = math.sqrt(ax * ax + ay * ay)

        if acc_norm > MAX_ACC:

            scale = MAX_ACC / acc_norm

            ax *= scale
            ay *= scale

        # BEZ CAŁKOWANIA PRĘDKOŚCI
        # u traktowane jako bezpośredni sygnał zadany

        self.vel_x = self.clamp(ax, -MAX_VEL, MAX_VEL)
        self.vel_y = self.clamp(ay, -MAX_VEL, MAX_VEL)

        # MAPOWANIE KOŁA

        vx_r = 0.70710678 * self.vel_x - 0.70710678 * self.vel_y
        vy_r = 0.70710678 * self.vel_x + 0.70710678 * self.vel_y

        V1 = -vy_r * math.cos(math.pi / 4)
        V2 = (-SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)
        V3 = (SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)

        # m/s -> rad/s

        w1 = V1 / r_k
        w2 = V2 / r_k
        w3 = V3 / r_k

        # MIN PWM FILTER

        w1 = self.min_command_filter(w1)
        w2 = self.min_command_filter(w2)
        w3 = self.min_command_filter(w3)

        # PUB

        self.msg.data[0] = float(-w3)
        self.msg.data[1] = float(-w2)
        self.msg.data[2] = float(-w1)

        self.pub.publish(self.msg)

        # LOGI

        self.log_counter += 1

        if self.log_counter >= 25:

            self.log_counter = 0

            self.get_logger().info(
                f'pitch={self.pitch:.4f} '
                f'roll={self.roll:.4f} '
                f'px={self.pos_x:.4f} '
                f'py={self.pos_y:.4f} '
                f'vx={self.vel_x:.4f} '
                f'vy={self.vel_y:.4f} '
                f'vx_meas={self.vel_x_meas:.4f} '
                f'vy_meas={self.vel_y_meas:.4f} '
                f'vx_r={vx_r:.4f} '
                f'vy_r={vy_r:.4f} '
                f'ax={ax:.4f} '
                f'ay={ay:.4f} '
                f'acc_norm={acc_norm:.4f} '
                f'w1={w1:.2f} '
                f'w2={w2:.2f} '
                f'w3={w3:.2f}'
            )

# MAIN

def main(args=None):

    rclpy.init(args=args)

    node = LqrBalanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()