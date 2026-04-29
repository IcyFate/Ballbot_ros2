#!/usr/bin/env python3

import math
import threading
import time
from collections import deque
from statistics import median

import pigpio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int32MultiArray


ENCODER_PIN_1A = 24
ENCODER_PIN_2A = 25
ENCODER_PIN_3A = 23

PPR = 480
WHEEL_DIAMETER = 0.048

GLITCH_US = 150
PUBLISH_RATE = 1000.0
TWO_PI = 2.0 * math.pi

PERIOD_EMA_ALPHA_LOW = 0.8
PERIOD_EMA_ALPHA_HIGH = 0.98

OMEGA_RATE_LIMIT = 150.0

MIN_VALID_PERIOD_S = 1e-5
MAX_VALID_PERIOD_S = 1.0

HIGH_SPEED_THRESHOLD = 20.0

# ===== NOWE =====
DT_ACCUM_MIN = 0.05     # minimalne okno dla delta_ticks (10 ms -> stabilność przy dużych prędkościach)
STOP_TIMEOUT = 0.1      # brak impulsów -> 0 prędkości


class EncoderOdomNode(Node):

    def __init__(self):
        super().__init__('encoder_odom_node')

        self.publisher = self.create_publisher(Float64MultiArray, 'wheel_state', 10)
        self.speed_pub = self.create_publisher(Float64, 'wheel1_speed', 10)

        self.dir_sub = self.create_subscription(
            Int32MultiArray, 'motor_direction', self.dir_callback, 10
        )

        self.direction = [0, 0, 0]

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

        self.last_edge_tick_us = [None, None, None]
        self.last_edge_time = [time.monotonic(), time.monotonic(), time.monotonic()]  # do detekcji zatrzymania

        self.period_hist = [deque(maxlen=3) for _ in range(3)]
        self.period_ema = [0.0, 0.0, 0.0]
        self.period_ema_init = [False, False, False]

        self.prev_ticks = [0, 0, 0]
        self.prev_time = time.monotonic()

        # ===== NOWE: akumulacja czasu dla delta_ticks =====
        self.dt_accum = [0.0, 0.0, 0.0]
        self.tick_accum = [0, 0, 0]

        self.omega_filtered = [0.0, 0.0, 0.0]

        self.cb = [
            self.pi.callback(self.encoder_pins[i],
                             pigpio.FALLING_EDGE,
                             self.make_encoder_callback(i))
            for i in range(3)
        ]

        self.ticks_per_rev = PPR
        self.wheel_circ = math.pi * WHEEL_DIAMETER

        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 16

        self.speed_msg = Float64()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_state
        )

    def dir_callback(self, msg):
        if len(msg.data) >= 3:
            self.direction = [int(msg.data[i]) for i in range(3)]

    def make_encoder_callback(self, motor_idx):
        def encoder_callback(gpio, level, tick):

            dir_i = self.direction[motor_idx]
            if dir_i == 0:
                return

            with self.lock:
                self.position_ticks[motor_idx] += dir_i
                self.last_edge_time[motor_idx] = time.monotonic()  # zapis czasu impulsu

                if self.last_edge_tick_us[motor_idx] is None:
                    self.last_edge_tick_us[motor_idx] = tick
                    return

                dt_us = pigpio.tickDiff(self.last_edge_tick_us[motor_idx], tick)
                self.last_edge_tick_us[motor_idx] = tick

                if dt_us <= 0:
                    return

                period = dt_us * 1e-6

                if period < MIN_VALID_PERIOD_S or period > MAX_VALID_PERIOD_S:
                    return

                self.period_hist[motor_idx].append(period)
                period_med = median(self.period_hist[motor_idx])

                omega_est = TWO_PI / (self.ticks_per_rev * period_med)

                alpha = PERIOD_EMA_ALPHA_HIGH if omega_est > HIGH_SPEED_THRESHOLD else PERIOD_EMA_ALPHA_LOW

                if not self.period_ema_init[motor_idx]:
                    self.period_ema[motor_idx] = period_med
                    self.period_ema_init[motor_idx] = True
                else:
                    self.period_ema[motor_idx] = (
                        alpha * period_med +
                        (1.0 - alpha) * self.period_ema[motor_idx]
                    )

                omega = TWO_PI / (self.ticks_per_rev * self.period_ema[motor_idx])
                self.omega_filtered[motor_idx] = dir_i * omega

        return encoder_callback

    def publish_state(self):

        now = time.monotonic()
        dt = now - self.prev_time
        self.prev_time = now

        with self.lock:
            ticks = self.position_ticks.copy()
            omega = self.omega_filtered.copy()
            last_edge_time = self.last_edge_time.copy()

        for i in range(3):

            # ===== FIX 1: detekcja zatrzymania =====
            if (now - last_edge_time[i]) > STOP_TIMEOUT:
                omega[i] = 0.0
                self.omega_filtered[i] = 0.0
                continue

            delta_ticks = ticks[i] - self.prev_ticks[i]

            # ===== FIX 2: akumulacja dla delta_ticks =====
            self.dt_accum[i] += dt
            self.tick_accum[i] += delta_ticks

            if self.dt_accum[i] >= DT_ACCUM_MIN:
                omega_dt = (self.tick_accum[i] / self.ticks_per_rev) * TWO_PI / self.dt_accum[i]

                # reset akumulatora
                self.dt_accum[i] = 0.0
                self.tick_accum[i] = 0

                # ===== miękkie mieszanie zamiast przełączania =====
                if abs(omega[i]) > HIGH_SPEED_THRESHOLD:
                    omega[i] = 0.5 * omega[i] + 0.5 * omega_dt

            # ===== RATE LIMIT =====
            max_step = OMEGA_RATE_LIMIT * dt
            diff = omega[i] - self.omega_filtered[i]

            if abs(diff) > max_step:
                omega[i] = self.omega_filtered[i] + math.copysign(max_step, diff)

            # lekkie wygładzenie końcowe
            omega[i] = 0.8 * omega[i] + 0.2 * self.omega_filtered[i]

        self.prev_ticks = ticks
        self.omega_filtered = omega

        now_s = self.get_clock().now().nanoseconds * 1e-9
        self.msg.data[0] = float(now_s)

        for i in range(3):
            rev = ticks[i] / self.ticks_per_rev
            angle = rev * TWO_PI
            distance = rev * self.wheel_circ

            linear_vel = omega[i] * self.wheel_circ / TWO_PI

            base = 1 + i * 5
            self.msg.data[base + 0] = float(ticks[i])
            self.msg.data[base + 1] = angle
            self.msg.data[base + 2] = distance
            self.msg.data[base + 3] = omega[i]
            self.msg.data[base + 4] = linear_vel

        self.speed_msg.data = float(omega[0])
        self.speed_pub.publish(self.speed_msg)

        self.publisher.publish(self.msg)

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