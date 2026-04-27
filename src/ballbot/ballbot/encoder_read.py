#!/usr/bin/env python3

import math
import threading
from collections import deque
import pigpio

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Float64, Float64MultiArray, Int32MultiArray  # type: ignore


ENCODER_PIN_1A = 24          # silnik1: 24   silnik2: 25    silnik3: 23
ENCODER_PIN_2A = 25
ENCODER_PIN_3A = 23

PPR = 480
WHEEL_DIAMETER = 0.048

GLITCH_US = 100
PUBLISH_RATE = 1000.0
VEL_WINDOW = 100   # liczba próbek w dłuższym oknie

TWO_PI = 2.0 * math.pi
OMEGA_EMA_ALPHA = 0.8  # filtr dolnoprzepustowy EMA; mniejsze = mocniejsze wygładzenie


class EncoderOdomNode(Node):

    def __init__(self):
        super().__init__('encoder_odom_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            'wheel_state',
            10
        )

        self.speed_pub = self.create_publisher(
            Float64,
            'wheel1_speed',
            10
        )

        self.dir_sub = self.create_subscription(
            Int32MultiArray,
            'motor_direction',
            self.dir_callback,
            10
        )

        self.direction = [0, 0, 0]  # kierunek dla każdego silnika osobno

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

        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_state
        )

        self.ticks_per_rev = PPR
        self.wheel_circ = math.pi * WHEEL_DIAMETER

        # bufory dla dłuższego okna czasowego
        self.time_buf = [deque(maxlen=VEL_WINDOW) for _ in range(3)]
        self.tick_buf = [deque(maxlen=VEL_WINDOW) for _ in range(3)]

        # jedna wiadomość używana wielokrotnie, żeby nie alokować przy każdej publikacji
        self.msg = Float64MultiArray()
        self.msg.data = [0.0] * 16  # [t, motor1..., motor2..., motor3...]

        # osobny publisher do wykresu prędkości silnika z enkodera na pinie 24
        self.speed_msg = Float64()
        self.wheel1_speed_ema = 0.0  # wygładzona prędkość koła 1
        self.wheel1_ema_initialized = False

    def dir_callback(self, msg):
        # odczyt kierunku dla 3 silników z Int32MultiArray
        if len(msg.data) >= 3:
            self.direction[0] = int(msg.data[0])
            self.direction[1] = int(msg.data[1])
            self.direction[2] = int(msg.data[2])

    def make_encoder_callback(self, motor_idx):
        def encoder_callback(gpio, level, tick):
            dir_i = self.direction[motor_idx]  # kierunek dla konkretnego silnika

            if dir_i == 0:
                return

            with self.lock:
                self.position_ticks[motor_idx] += dir_i  # inkrementacja zgodnie z kierunkiem
        return encoder_callback

    def publish_state(self):

        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        with self.lock:
            ticks_1 = self.position_ticks[0]
            ticks_2 = self.position_ticks[1]
            ticks_3 = self.position_ticks[2]

        ticks_list = [ticks_1, ticks_2, ticks_3]

        # zapis do jednej wiadomości
        self.msg.data[0] = float(now_s)

        wheel1_omega_raw = 0.0

        for i in range(3):
            ticks = ticks_list[i]

            # pozycja absolutna
            rev = ticks / self.ticks_per_rev
            angle = rev * TWO_PI
            distance = rev * self.wheel_circ

            # aktualizacja bufora dłuższego okna
            self.time_buf[i].append(now_s)
            self.tick_buf[i].append(ticks)

            # prędkość liczona z dłuższego okna:
            # omega = 2*pi * delta_ticks / (PPR * delta_t)
            if len(self.time_buf[i]) >= 2:
                dt_window = self.time_buf[i][-1] - self.time_buf[i][0]
                delta_ticks = self.tick_buf[i][-1] - self.tick_buf[i][0]

                if dt_window > 0.0:
                    vel_rev = (delta_ticks / self.ticks_per_rev) / dt_window
                else:
                    vel_rev = 0.0
            else:
                vel_rev = 0.0

            omega_raw = vel_rev * TWO_PI
            linear_vel_raw = vel_rev * self.wheel_circ

            # filtr dolnoprzepustowy EMA na prędkości
            if i == 0:
                if not self.wheel1_ema_initialized:
                    self.wheel1_speed_ema = omega_raw
                    self.wheel1_ema_initialized = True
                else:
                    self.wheel1_speed_ema = (
                        OMEGA_EMA_ALPHA * omega_raw +
                        (1.0 - OMEGA_EMA_ALPHA) * self.wheel1_speed_ema
                    )
                omega = self.wheel1_speed_ema
                wheel1_omega_raw = omega_raw
            else:
                # osobne wygładzanie dla pozostałych kół bez dodatkowych topiców
                base_omega_ema_name = f"_omega_ema_{i}"
                if not hasattr(self, base_omega_ema_name):
                    setattr(self, base_omega_ema_name, omega_raw)
                    setattr(self, f"_omega_ema_init_{i}", True)
                    omega = omega_raw
                else:
                    prev_omega = getattr(self, base_omega_ema_name)
                    omega = (
                        OMEGA_EMA_ALPHA * omega_raw +
                        (1.0 - OMEGA_EMA_ALPHA) * prev_omega
                    )
                    setattr(self, base_omega_ema_name, omega)

            linear_vel = omega * self.wheel_circ / TWO_PI

            base = 1 + i * 5
            self.msg.data[base + 0] = float(ticks)
            self.msg.data[base + 1] = angle
            self.msg.data[base + 2] = distance
            self.msg.data[base + 3] = omega
            self.msg.data[base + 4] = linear_vel

        # wygładzona prędkość koła 1 na wykres
        self.speed_msg.data = float(self.wheel1_speed_ema)
        self.speed_pub.publish(self.speed_msg)

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