#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


def cov_3x3_from_msg(flat_cov, fallback_var: float) -> np.ndarray:
    """
    Zamienia tablicę kowariancji 3×3 z wiadomości IMU na macierz NumPy.
    Jeśli kowariancja jest nieznana albo pusta, zwraca zastępczą macierz diagonalną.
    """
    cov = np.asarray(flat_cov, dtype=float).reshape(3, 3)

    if cov[0, 0] == -1.0 or np.allclose(cov, 0.0):
        return np.eye(3) * fallback_var

    cov = 0.5 * (cov + cov.T)
    cov = cov + np.eye(3) * 1e-12
    return cov


class ImuKalmanSmoother:
    """
    Filtr Kalmana dla sześciu kanałów:
    x = [wx, wy, wz, ax, ay, az]^T

    To jest filtr wygładzający, a nie model orientacji ani pozycji.
    """

    def __init__(
        self,
        process_var_gyro: float = 0.05,
        process_var_acc: float = 0.5,
        fallback_gyro_meas_var: float = 0.05,
        fallback_acc_meas_var: float = 0.5,
    ):
        self.x = np.zeros(6, dtype=float)  # Stan początkowy.
        self.P = np.eye(6, dtype=float) * 1000.0  # Duża niepewność początkowa.

        self.F = np.eye(6, dtype=float)  # Model losowego spaceru.
        self.H = np.eye(6, dtype=float)  # Pomiar obserwuje stan bezpośrednio.
        self.W = np.eye(6, dtype=float)  # Mapowanie szumu procesu.
        self.G = np.zeros((6, 1), dtype=float)  # Brak jawnego wejścia sterującego.

        self.Q_base = np.diag(
            [process_var_gyro] * 3 + [process_var_acc] * 3
        ).astype(float)  # Kowariancja szumu procesu.

        self.fallback_gyro_meas_var = fallback_gyro_meas_var
        self.fallback_acc_meas_var = fallback_acc_meas_var

        self.I = np.eye(6, dtype=float)

    def predict(self, dt: float):
        dt = max(float(dt), 1e-6)

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.W @ (self.Q_base * dt) @ self.W.T

    def update(self, z: np.ndarray, R: np.ndarray):
        z = np.asarray(z, dtype=float).reshape(6)
        R = np.asarray(R, dtype=float).reshape(6, 6)

        y = z - self.H @ self.x  # Innowacja, czyli błąd pomiędzy pomiarem i predykcją.
        S = self.H @ self.P @ self.H.T + R  # Kowariancja innowacji.
        K = self.P @ self.H.T @ np.linalg.pinv(S)  # Wzmocnienie Kalmana.

        self.x = self.x + K @ y  # Korekta stanu.
        self.P = (self.I - K @ self.H) @ self.P  # Korekta niepewności.

    def step(self, msg: Imu, dt: float):
        self.predict(dt)

        z = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ], dtype=float)

        R_gyro = cov_3x3_from_msg(
            msg.angular_velocity_covariance,
            self.fallback_gyro_meas_var
        )
        R_acc = cov_3x3_from_msg(
            msg.linear_acceleration_covariance,
            self.fallback_acc_meas_var
        )

        R = np.block([
            [R_gyro, np.zeros((3, 3))],
            [np.zeros((3, 3)), R_acc],
        ])

        self.update(z, R)
        return z, self.x.copy(), R


class ImuKalmanPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_publisher')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/imu/kalman_smoothed',
            10
        )

        self.filter = ImuKalmanSmoother()

        self.last_stamp_sec = None

    def imu_callback(self, msg: Imu):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        if self.last_stamp_sec is None:
            self.last_stamp_sec = stamp_sec
            return

        dt = stamp_sec - self.last_stamp_sec
        if dt <= 0.0 or not math.isfinite(dt):
            dt = 0.01

        self.last_stamp_sec = stamp_sec

        raw, filt, _ = self.filter.step(msg, dt)

        # Publikujemy czas, 6 wartości surowych i 6 wartości po filtrze.
        out = Float64MultiArray()
        out.data = [stamp_sec] + raw.tolist() + filt.tolist()

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = ImuKalmanPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()