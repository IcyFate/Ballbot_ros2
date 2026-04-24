#!/usr/bin/env python3

import math
import threading
from collections import deque
import pigpio

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64MultiArray, Int32  # type: ignore


ENCODER_PIN_1A = 24          # silnik1: 24   silnik2: 25    silnik3: 23
ENCODER_PIN_2A = 25
ENCODER_PIN_3A = 23

PPR = 480
WHEEL_DIAMETER = 0.048

GLITCH_US = 100
PUBLISH_RATE = 1000.0
VEL_WINDOW = 20   # liczba próbek w moving average

TWO_PI = 2.0 * math.pi


class EncoderOdomNode(Node):

    def __init__(self):
        super().__init__('encoder_odom_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            'wheel_state',
            10
        )

        self.dir_sub = self.create_subscription(
            Int32,
            'motor_direction',
            self.dir_callback,
            10
        )

        self.direction = 0

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running")

        self.encoder_pins = [ENCODER_PIN_1A, ENCODER_PIN_2A, ENCODER_PIN_3A]

        for pin in self.encoder_pins:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            self.pi.set_glitch_filter(pin, GLITCH_US)

        self.lock = threading.Lock()

        self.position_ticks = [0, 0, 0]

        self.cb = [
            self.pi.callback(
                self.encoder_pins[0],
                pigpio.FALLING_EDGE,
                self.make_encoder_callback(0)
            ),
            self.pi.callback(
                self.encoder_pins[1],
                pigpio.FALLING_EDGE,
                self.make_encoder_callback(1)
            ),
            self.pi.callback(
                self.encoder_pins[2],
                pigpio.FALLING_EDGE,
                self.make_encoder_callback(2)
            )
        ]

        self.prev_ticks = [0, 0, 0]
        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_state
        )

        self.ticks_per_rev = PPR
        self.wheel_circ = math.pi * WHEEL_DIAMETER

        # bufory moving average
        self.dt_buf = [deque(maxlen=VEL_WINDOW) for _ in range(3)]
        self.tick_buf = [deque(maxlen=VEL_WINDOW) for _ in range(3)]

        # sumy ruchome do szybszego liczenia średniej bez sum(dt_buf) za każdym razem
        self.sum_dt = [0.0, 0.0, 0.0]
        self.sum_ticks = [0.0, 0.0, 0.0]

        # jedna wiadomość używana wielokrotnie, żeby nie alokować przy każdej publikacji
        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 16  # [t, motor1..., motor2..., motor3...]

    def dir_callback(self, msg):
        self.direction = int(msg.data)

    def make_encoder_callback(self, motor_idx):
        def encoder_callback(gpio, level, tick):
            if self.direction == 0:
                return
            with self.lock:
                self.position_ticks[motor_idx] += self.direction
        return encoder_callback

    def publish_state(self):

        now = self.get_clock().now()

        with self.lock:
            ticks_1 = self.position_ticks[0]
            ticks_2 = self.position_ticks[1]
            ticks_3 = self.position_ticks[2]

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        ticks_list = [ticks_1, ticks_2, ticks_3]

        # aktualizacja buforów moving average
        for i in range(3):
            delta_ticks = ticks_list[i] - self.prev_ticks[i]

            if len(self.dt_buf[i]) == VEL_WINDOW:
                self.sum_dt[i] -= self.dt_buf[i][0]
                self.sum_ticks[i] -= self.tick_buf[i][0]

            self.dt_buf[i].append(dt)
            self.tick_buf[i].append(delta_ticks)

            self.sum_dt[i] += dt
            self.sum_ticks[i] += delta_ticks

        # zapis do jednej wiadomości
        self.msg.data[0] = float(now.nanoseconds * 1e-9)

        for i in range(3):
            ticks = ticks_list[i]

            # pozycja absolutna
            rev = ticks / self.ticks_per_rev
            angle = rev * TWO_PI
            distance = rev * self.wheel_circ

            # prędkość z moving average
            if self.sum_dt[i] > 0:
                vel_rev = (self.sum_ticks[i] / self.ticks_per_rev) / self.sum_dt[i]
            else:
                vel_rev = 0.0

            omega = vel_rev * TWO_PI
            linear_vel = vel_rev * self.wheel_circ

            base = 1 + i * 5
            self.msg.data[base + 0] = float(ticks)
            self.msg.data[base + 1] = angle
            self.msg.data[base + 2] = distance
            self.msg.data[base + 3] = omega
            self.msg.data[base + 4] = linear_vel

            self.prev_ticks[i] = ticks

        self.publisher.publish(self.msg)

        self.prev_time = now

    def destroy_node(self):
        for c in self.cb:
            c.cancel()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()