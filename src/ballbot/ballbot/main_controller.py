#!/usr/bin/env python3

import math
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# MODEL

r_k = 0.024

# LQR

K1 = -15
K2 = -3
K3 = 2
K4 = 5

K1_STEP = 1.0
K2_STEP = 1.0
K3_STEP = 0.2
K4_STEP = 0.2

# ZNAKI SPRZĘŻENIA TRANSLACYJNEGO

POS_SIGN_X = -1.0
POS_SIGN_Y = -1.0
VEL_SIGN_X = -1.0
VEL_SIGN_Y = -1.0

# SILNIKI

MIN_COMMAND_RAD = 0

# GEOMETRIA

R_BALL = 0.125

SQRT3_2 = 0.86602540378
SQRT2_2 = 0.70710678

MAX_ACC = 5
MAX_VEL = 1

ANGLE_DEADBAND = 0.01
RATE_DEADBAND = 0.015
POSITION_DEADBAND = 0.05
VELOCITY_DEADBAND = 0.01


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

        self.pos_x_ref = None
        self.pos_y_ref = None

        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0

        self.k1 = float(K1)
        self.k2 = float(K2)
        self.k3 = float(K3)
        self.k4 = float(K4)

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

        self._keyboard_stop = threading.Event()
        self._keyboard_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self._keyboard_thread.start()

        self.get_logger().info('LQR tuning node started')
        self.get_logger().info(
            'Keys: q/a -> K1 +/-1, w/s -> K2 +/-1, '
            'e/d -> K3 +/-0.2, r/f -> K4 +/-0.2, x -> exit'
        )
        self.print_status()

    def print_status(self):
        self.get_logger().info(
            f'ACTUAL: K1={self.k1:.2f}, K2={self.k2:.2f}, '
            f'K3={self.k3:.2f}, K4={self.k4:.2f}'
        )

    def keyboard_loop(self):
        tty_file = None
        old_settings = None

        try:
            try:
                tty_file = open('/dev/tty', 'r')
            except OSError:
                tty_file = sys.stdin

            fd = tty_file.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)

            while not self._keyboard_stop.is_set():
                rlist, _, _ = select.select([tty_file], [], [], 0.1)
                if not rlist:
                    continue

                ch = tty_file.read(1)
                if not ch:
                    continue

                if ch == 'q':
                    self.k1 += K1_STEP
                elif ch == 'a':
                    self.k1 -= K1_STEP
                elif ch == 'w':
                    self.k2 += K2_STEP
                elif ch == 's':
                    self.k2 -= K2_STEP
                elif ch == 'e':
                    self.k3 += K3_STEP
                elif ch == 'd':
                    self.k3 -= K3_STEP
                elif ch == 'r':
                    self.k4 += K4_STEP
                elif ch == 'f':
                    self.k4 -= K4_STEP
                elif ch == 'x':
                    self.get_logger().info('Exit requested from keyboard')
                    self._keyboard_stop.set()
                    rclpy.shutdown()
                    break
                else:
                    continue

                self.print_status()

        except Exception as e:
            self.get_logger().error(f'Keyboard thread error: {e}')
        finally:
            if old_settings is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

            if tty_file is not None and tty_file is not sys.stdin:
                tty_file.close()

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

        s1_raw = float(d[3])
        s2_raw = float(d[8])
        s3_raw = float(d[13])

        u1_raw = float(d[5])
        u2_raw = float(d[10])
        u3_raw = float(d[15])

        s1 = s3_raw
        s2 = s2_raw
        s3 = s1_raw

        u1 = u3_raw
        u2 = u2_raw
        u3 = u1_raw

        c = SQRT2_2

        psi_pos = -s1 / (R_BALL * c)
        phi_pos = (s3 - s2) / (2.0 * SQRT3_2 * R_BALL * c)
        self.pos_x_meas = R_BALL * c * (phi_pos + psi_pos)
        self.pos_y_meas = R_BALL * c * (-phi_pos + psi_pos)

        psi_vel = -u1 / (R_BALL * c)
        phi_vel = (u3 - u2) / (2.0 * SQRT3_2 * R_BALL * c)
        self.vel_x_meas = R_BALL * c * (phi_vel + psi_vel)
        self.vel_y_meas = R_BALL * c * (-phi_vel + psi_vel)

        if self.pos_x_ref is None:
            self.pos_x_ref = self.pos_x_meas
            self.pos_y_ref = self.pos_y_meas

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

        theta_x = self.deadband(self.pitch, ANGLE_DEADBAND)
        theta_y = self.deadband(self.roll, ANGLE_DEADBAND)

        theta_dot_x = self.deadband(self.pitch_rate, RATE_DEADBAND)
        theta_dot_y = self.deadband(self.roll_rate, RATE_DEADBAND)

        self.pos_x = self.pos_x_meas
        self.pos_y = self.pos_y_meas

        self.vel_x = self.vel_x_meas
        self.vel_y = self.vel_y_meas

        if self.pos_x_ref is None or self.pos_y_ref is None:
            px_fb = 0.0
            py_fb = 0.0
        else:
            px_fb = self.deadband(self.pos_x_meas - self.pos_x_ref, POSITION_DEADBAND)
            py_fb = self.deadband(self.pos_y_meas - self.pos_y_ref, POSITION_DEADBAND)

        vx_fb = self.deadband(self.vel_x_meas, VELOCITY_DEADBAND)
        vy_fb = self.deadband(self.vel_y_meas, VELOCITY_DEADBAND)

        ax = -(self.k1 * theta_x + self.k2 * theta_dot_x
               + self.k3 * (POS_SIGN_X * px_fb)
               + self.k4 * (VEL_SIGN_X * vx_fb))

        ay = -(self.k1 * theta_y + self.k2 * theta_dot_y
               + self.k3 * (POS_SIGN_Y * py_fb)
               + self.k4 * (VEL_SIGN_Y * vy_fb))

        acc_norm = math.sqrt(ax * ax + ay * ay)

        if acc_norm > MAX_ACC:
            scale = MAX_ACC / acc_norm
            ax *= scale
            ay *= scale

        self.cmd_vel_x += ax * dt
        self.cmd_vel_y += ay * dt

        self.cmd_vel_x = self.clamp(self.cmd_vel_x, -MAX_VEL, MAX_VEL)
        self.cmd_vel_y = self.clamp(self.cmd_vel_y, -MAX_VEL, MAX_VEL)

        if abs(px_fb) < POSITION_DEADBAND and abs(py_fb) < POSITION_DEADBAND and \
           abs(vx_fb) < VELOCITY_DEADBAND and abs(vy_fb) < VELOCITY_DEADBAND and \
           abs(theta_x) < ANGLE_DEADBAND and abs(theta_y) < ANGLE_DEADBAND and \
           abs(theta_dot_x) < RATE_DEADBAND and abs(theta_dot_y) < RATE_DEADBAND:
            self.cmd_vel_x = 0.0
            self.cmd_vel_y = 0.0

        vx_r = 0.70710678 * self.cmd_vel_x - 0.70710678 * self.cmd_vel_y
        vy_r = 0.70710678 * self.cmd_vel_x + 0.70710678 * self.cmd_vel_y

        V1 = -vy_r * math.cos(math.pi / 4)
        V2 = (-SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)
        V3 = (SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)

        w1 = V1 / r_k
        w2 = V2 / r_k
        w3 = V3 / r_k

        w1 = self.min_command_filter(w1)
        w2 = self.min_command_filter(w2)
        w3 = self.min_command_filter(w3)

        self.msg.data[0] = float(-w3)
        self.msg.data[1] = float(-w2)
        self.msg.data[2] = float(-w1)

        self.pub.publish(self.msg)

        self.log_counter += 1

        if self.log_counter >= 100:
            self.log_counter = 0

            self.get_logger().info(
                f'K1={self.k1:.2f} '
                f'K2={self.k2:.2f} '
                f'K3={self.k3:.2f} '
                f'K4={self.k4:.2f} '
                f'pitch={self.pitch:.4f} '
                f'roll={self.roll:.4f} '
                f'px_used={self.pos_x:.4f} '
                f'py_used={self.pos_y:.4f} '
                f'vx_meas={self.vel_x_meas:.4f} '
                f'vy_meas={self.vel_y_meas:.4f} '
                f'ax={ax:.4f} '
                f'ay={ay:.4f} '
                f'w1={w1:.2f} '
                f'w2={w2:.2f} '
                f'w3={w3:.2f}'
            )

    def stop_keyboard(self):
        self._keyboard_stop.set()


# MAIN

def main(args=None):
    rclpy.init(args=args)

    node = LqrBalanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_keyboard()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
