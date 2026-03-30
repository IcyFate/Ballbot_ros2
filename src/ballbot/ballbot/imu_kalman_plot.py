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

        self.max_points = 600
        self.time_window = 10.0  # sekundy

        self.t_hist = deque(maxlen=self.max_points)
        self.filt_hist = [deque(maxlen=self.max_points) for _ in range(6)]

        self.channel_names = [
            'gyro x', 'gyro y', 'gyro z',
            'acc x', 'acc y', 'acc z'
        ]

        self.start_time_sec = None

        plt.ion()
        self.fig, self.axes = plt.subplots(
            2, 3,
            figsize=(15, 7),
            sharex=True
        )
        self.axes = self.axes.flatten()

        self.lines = []

        for i, ax in enumerate(self.axes):
            line, = ax.plot([], [], linewidth=1.2)

            ax.set_title(self.channel_names[i])
            ax.grid(True, alpha=0.3)

            self.lines.append(line)

        self.fig.suptitle('Kalman smoothed IMU data')
        self.fig.tight_layout()

        self.timer = self.create_timer(0.05, self.update_plot)

    def callback(self, msg: Float64MultiArray):
        data = np.asarray(msg.data, dtype=float)

        # oczekiwany format:
        # [timestamp, gx, gy, gz, ax, ay, az]
        if data.size < 7:
            return

        stamp_sec = float(data[0])

        if self.start_time_sec is None:
            self.start_time_sec = stamp_sec

        t_rel = stamp_sec - self.start_time_sec
        self.t_hist.append(t_rel)

        filt = data[1:7]

        for i in range(6):
            self.filt_hist[i].append(float(filt[i]))

    def update_plot(self):
        if len(self.t_hist) < 2:
            return

        t = np.asarray(self.t_hist, dtype=float)

        for i, ax in enumerate(self.axes):
            y = np.asarray(self.filt_hist[i], dtype=float)

            self.lines[i].set_data(t, y)

            ax.relim()
            ax.autoscale_view()

            t_max = t[-1]
            if t_max > self.time_window:
                ax.set_xlim(t_max - self.time_window, t_max)
            else:
                ax.set_xlim(0.0, self.time_window)

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