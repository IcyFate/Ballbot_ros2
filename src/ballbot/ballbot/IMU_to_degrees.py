#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# from geometry_msgs.msg import Vector3Stamped  # WYŁĄCZONE – dodatkowy narzut CPU

import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
from adafruit_lsm6ds import Rate


BURST_START_REG = 0x22  # adres pierwszego rejestru danych IMU (gyro X LSB)
BURST_LEN = 12          # liczba bajtów: gx,gy,gz,ax,ay,az (6 * int16)

GYRO_LSB_TO_RAD_S = (0.00875 * math.pi / 180.0)  # przelicznik z LSB -> rad/s (±250 dps)
ACC_LSB_TO_MS2 = (0.244e-3 * 9.80665)            # przelicznik z LSB -> m/s^2 (±8g)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))  # stabilne zawijanie kąta do [-pi, pi]


class TiltEkf:
    def __init__(self):
        # wektor stanu: [roll, pitch, bx, by, bz]
        self.roll = 0.0
        self.pitch = 0.0
        self.bx = 0.0
        self.by = 0.0
        self.bz = 0.0

        # uproszczona "kowariancja" – tylko skalary zamiast macierzy 5x5
        self.P = 100.0  # macierz kowariancji (5x5), duża niepewność początkowa
        self.I = 1.0    # macierz jednostkowa (do aktualizacji P)

        self.q_angle = 0.002   # szum procesu dla kątów (jak szybko mogą się zmieniać)
        self.q_bias = 5e-5     # szum procesu dla biasów (wolny dryft)

        self.R = 1.5e-6  # szum pomiaru (roll, pitch z akcelerometru)

        self.initialized = False  # flaga inicjalizacji filtru

    def initialize_from_acc(self, acc):
        ax, ay, az = acc  # przyspieszenia w osiach IMU

        self.roll = math.atan2(ay, az)  # roll z grawitacji
        self.pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))  # pitch z grawitacji

        self.bx = 0.0  # biasy gyro ustawione na 0
        self.by = 0.0
        self.bz = 0.0

        self.initialized = True  # filtr gotowy do pracy

    def predict(self, gyro, dt):
        gx, gy, gz = gyro  # pomiar żyroskopu [rad/s]

        phi = self.roll     # roll
        theta = self.pitch  # pitch

        # prędkości kątowe po korekcji biasu
        p = gx - self.bx  # prędkość kątowa wokół X (skorygowana o bias)
        q = gy - self.by  # prędkość wokół Y
        r = gz - self.bz  # prędkość wokół Z

        sphi = math.sin(phi)   # sin(roll)
        cphi = math.cos(phi)   # cos(roll)
        tth = math.tan(theta)  # tan(pitch)

        # równania kinematyki Eulera (nieliniowe)
        phi_dot = p + sphi * tth * q + cphi * tth * r     # pochodna roll
        theta_dot = cphi * q - sphi * r                   # pochodna pitch

        self.roll += dt * phi_dot   # integracja roll
        self.pitch += dt * theta_dot # integracja pitch

        # uproszczony model niepewności (bez Jacobianu)
        self.P += (self.q_angle * dt)  # zwiększenie niepewności

    def update_from_acc(self, acc):
        ax, ay, az = acc  # przyspieszenia

        roll_acc = math.atan2(ay, az)  # pomiar roll z akcelerometru
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))  # pomiar pitch

        y0 = wrap_angle(roll_acc - self.roll)  # błąd pomiaru roll
        y1 = pitch_acc - self.pitch            # błąd pomiaru pitch

        k = 0.15  # stałe wzmocnienie (zamiast pełnego Kalmana – szybciej)

        self.roll += k * y0  # korekta roll
        self.pitch += k * y1  # korekta pitch

    def step(self, acc, gyro, dt):
        if not self.initialized:
            self.initialize_from_acc(acc)  # inicjalizacja z grawitacji

        self.predict(gyro, dt)        # predykcja z gyro
        self.update_from_acc(acc)     # korekta z akcelerometru

        gx, gy, gz = gyro

        # prędkości kątowe po korekcji biasu
        p = gx - self.bx  # prędkość kątowa X (po korekcji)
        q = gy - self.by
        r = gz - self.bz

        return self.roll, self.pitch, p, q, r, self.bx, self.by, self.bz  # zwracamy cały stan


class ImuKalmanNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/imu/kalman_state',
            10
        )  # jedyny publisher – minimalny narzut

        self.i2c = busio.I2C(board.SCL, board.SDA)  # magistrala I2C

        self.imu = LSM6DSO32(self.i2c, address=0x6A)  # inicjalizacja sensora

        self.imu.accelerometer_data_rate = Rate.RATE_208_HZ  # niższy ODR = mniej szumu
        self.imu.gyro_data_rate = Rate.RATE_208_HZ

        self.filter = TiltEkf()  # instancja filtru

        self.last_time = self.get_clock().now()  # czas poprzedniej iteracji

        self.rx = bytearray(BURST_LEN)  # bufor na dane z I2C

        self.msg = Float64MultiArray()  # wiadomość publikowana

        self.timer = self.create_timer(0.001, self.loop)  # 1 kHz pętla

    def read_burst(self):
        self.i2c.writeto_then_readfrom(
            0x6A,
            bytes([BURST_START_REG]),
            self.rx
        )  # szybki odczyt 12 bajtów jednym transferem

        gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = struct.unpack('<hhhhhh', self.rx)  # unpack int16

        return (
            (
                ax_raw * ACC_LSB_TO_MS2,  # ax w m/s^2
                ay_raw * ACC_LSB_TO_MS2,
                az_raw * ACC_LSB_TO_MS2,
            ),
            (
                gx_raw * GYRO_LSB_TO_RAD_S,  # gx w rad/s
                gy_raw * GYRO_LSB_TO_RAD_S,
                gz_raw * GYRO_LSB_TO_RAD_S,
            )
        )

    def loop(self):
        now = self.get_clock().now()  # aktualny czas

        dt = (now - self.last_time).nanoseconds * 1e-9  # delta czasu w sekundach
        self.last_time = now

        if dt <= 0.0:
            dt = 0.001  # fallback

        try:
            acc, gyro = self.read_burst()  # odczyt IMU
        except OSError:
            return  # brak logowania = szybciej

        roll, pitch, p, q, r, bx, by, bz = self.filter.step(acc, gyro, dt)  # filtr

        # kompensacja stałego błędu (offset montażu / bias)
        roll += 0.0261799388   # +1.5°
        pitch -= 0.0087266463  # -0.5°

        # publikacja minimalna (bez timestampu, bez headerów)
        self.msg.data = [
            roll, pitch, p, q, r, bx, by, bz
        ]

        self.publisher.publish(self.msg)  # główny bottleneck – ROS publish


def main():
    rclpy.init()
    node = ImuKalmanNode()

    try:
        rclpy.spin(node)  # główna pętla ROS
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()