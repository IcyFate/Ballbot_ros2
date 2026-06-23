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
from sensor_msgs.msg import Imu

# MODEL

r_k = 0.024

# PID

KP = 5
KI = 0.2
KD = 0.6

KP_STEP = 1.0
KI_STEP = 0.2
KD_STEP = 0.2

I_LIMIT = 1.0

# SILNIKI

MIN_COMMAND_RAD = 2.2
MAX_W_RAD = 35.0

# GEOMETRIA

SQRT3_2 = 0.86602540378

ANGLE_DEADBAND = 0.0


class AnglePid:
    def __init__(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.derivative = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.derivative = 0.0
        self.initialized = False

    def update(self, error, dt, kp, ki, kd):
        if not self.initialized:
            self.prev_error = error
            self.derivative = 0.0
            self.initialized = True

        self.integral += error * dt
        self.integral = max(-I_LIMIT, min(I_LIMIT, self.integral))

        self.derivative = (error - self.prev_error) / dt
        self.prev_error = error

        temp = kp * error + ki * self.integral + kd * self.derivative

        return temp

    def anti_windup(self, saturated_output, kp, ki, kd):
        if ki == 0.0:
            return

        self.integral = (
            saturated_output
            - kp * self.prev_error
            - kd * self.derivative
        ) / ki

        self.integral = max(-I_LIMIT, min(I_LIMIT, self.integral))


class PidBalanceController(Node):

    def __init__(self):
        super().__init__('pid_balance_controller')

        self.roll = 0.0
        self.pitch = 0.0

        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0

        self.kp = float(KP)
        self.ki = float(KI)
        self.kd = float(KD)

        self.pid_x = AnglePid()
        self.pid_y = AnglePid()

        self.last_time = self.get_clock().now()
        self.log_counter = 0

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
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

        self.get_logger().info('PID balance controller started')
        self.get_logger().info(
            'Keys: q/a -> Kp +/-1, w/s -> Ki +/-1, '
            'e/d -> Kd +/-0.2, x -> exit'
        )
        self.print_status()

    def print_status(self):
        self.get_logger().info(
            f'ACTUAL: Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}'
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
                    self.kp += KP_STEP
                elif ch == 'a':
                    self.kp -= KP_STEP
                elif ch == 'w':
                    self.ki += KI_STEP
                elif ch == 's':
                    self.ki -= KI_STEP
                elif ch == 'e':
                    self.kd += KD_STEP
                elif ch == 'd':
                    self.kd -= KD_STEP
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
        q = msg.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            self.pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.pitch = math.asin(sinp)

    def deadband(self, x, threshold):
        if abs(x) < threshold:
            return 0.0

        return x

    def min_command_filter(self, x):
        if abs(x) < MIN_COMMAND_RAD:
            return 0.0

        return x

    def limit_wheels(self, w1, w2, w3):
        max_w = max(abs(w1), abs(w2), abs(w3))
        scale = 1.0

        if max_w > MAX_W_RAD:
            scale = MAX_W_RAD / max_w
            w1 *= scale
            w2 *= scale
            w3 *= scale

        return w1, w2, w3, scale

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

        if theta_x == 0.0 and theta_y == 0.0:
            self.pid_x.reset()
            self.pid_y.reset()

            self.cmd_vel_x = 0.0
            self.cmd_vel_y = 0.0
        else:
            self.cmd_vel_x = self.pid_x.update(theta_x, dt, self.kp, self.ki, self.kd)
            self.cmd_vel_y = self.pid_y.update(theta_y, dt, self.kp, self.ki, self.kd)

        vx_r = 0.70710678 * self.cmd_vel_x - 0.70710678 * self.cmd_vel_y
        vy_r = 0.70710678 * self.cmd_vel_x + 0.70710678 * self.cmd_vel_y

        V1 = -vy_r * math.cos(math.pi / 4)
        V2 = (-SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)
        V3 = (SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)

        w1 = V1 / r_k
        w2 = V2 / r_k
        w3 = V3 / r_k

        w1, w2, w3, wheel_scale = self.limit_wheels(w1, w2, w3)

        if wheel_scale < 1.0:
            self.cmd_vel_x *= wheel_scale
            self.cmd_vel_y *= wheel_scale
            vx_r *= wheel_scale
            vy_r *= wheel_scale

            self.pid_x.anti_windup(self.cmd_vel_x, self.kp, self.ki, self.kd)
            self.pid_y.anti_windup(self.cmd_vel_y, self.kp, self.ki, self.kd)

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
                f'pitch={self.pitch:.4f} '
                f'roll={self.roll:.4f} '
                f'cmd_x={self.cmd_vel_x:.4f} '
                f'cmd_y={self.cmd_vel_y:.4f} '
                f'vx={vx_r:.4f} '
                f'vy={vy_r:.4f} '
                f'w1={w1:.2f} '
                f'w2={w2:.2f} '
                f'w3={w3:.2f}'
            )

    def stop_keyboard(self):
        self._keyboard_stop.set()


def main(args=None):
    rclpy.init(args=args)

    node = PidBalanceController()

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