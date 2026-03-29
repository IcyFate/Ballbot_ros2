#!/usr/bin/env python3
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt


class ImuKalmanPlotterNode(Node):
    def __init__(self):
        super().__init__('imu_kalman_plotter')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/imu/kalman_smoothed',
            self.callback,
            10
        )

        self.max_points = 600  # Liczba punktów w buforze wykresu.
        self.t_hist = deque(maxlen=self.max_points)

        self.raw_hist = [deque(maxlen=self.max_points) for _ in range(6)]
        self.filt_hist = [deque(maxlen=self.max_points) for _ in range(6)]

        self.channel_names = [
            'gyro x', 'gyro y', 'gyro z',
            'acc x', 'acc y', 'acc z'
        ]

        self.start_time_sec = None  # Pierwszy czas do osi względnej.

        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
        self.axes = self.axes.flatten()

        self.raw_lines = []
        self.filt_lines = []

        for i, ax in enumerate(self.axes):
            # Dane surowe jako kropki, bez łączenia linią.
            raw_line, = ax.plot([], [], linestyle='None', marker='o', markersize=3, label='raw')
            # Dane po filtrze jako linia.
            filt_line, = ax.plot([], [], label='kalman')

            ax.set_title(self.channel_names[i])
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right')

            self.raw_lines.append(raw_line)
            self.filt_lines.append(filt_line)

        self.fig.suptitle('IMU raw data and Kalman-smoothed data')
        self.fig.tight_layout()

        # Timer do odświeżania wykresu, niezależnie od tempa napływu wiadomości.
        self.timer = self.create_timer(0.05, self.update_plot)

    def callback(self, msg: Float64MultiArray):
        data = np.asarray(msg.data, dtype=float)

        # Oczekiwany format:
        # [time, raw_0..raw_5, filt_0..filt_5]
        if data.size < 13:
            return

        stamp_sec = float(data[0])

        if self.start_time_sec is None:
            self.start_time_sec = stamp_sec

        t_rel = stamp_sec - self.start_time_sec
        self.t_hist.append(t_rel)

        raw = data[1:7]
        filt = data[7:13]

        for i in range(6):
            self.raw_hist[i].append(float(raw[i]))
            self.filt_hist[i].append(float(filt[i]))

    def update_plot(self):
        if len(self.t_hist) < 2:
            return

        t = np.asarray(self.t_hist, dtype=float)

        for i, ax in enumerate(self.axes):
            raw_y = np.asarray(self.raw_hist[i], dtype=float)
            filt_y = np.asarray(self.filt_hist[i], dtype=float)

            self.raw_lines[i].set_data(t, raw_y)
            self.filt_lines[i].set_data(t, filt_y)

            ax.relim()
            ax.autoscale_view()

            t_max = t[-1]
            if t_max > 10.0:
                ax.set_xlim(t_max - 10.0, t_max)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = ImuKalmanPlotterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()