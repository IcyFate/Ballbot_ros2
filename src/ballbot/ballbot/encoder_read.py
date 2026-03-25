import math
import threading
import pigpio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32


ENCODER_PIN_A = 24

PPR = 480
WHEEL_DIAMETER = 0.048

GLITCH_US = 100
PUBLISH_RATE = 100.0
VEL_WINDOW = 20   # liczba próbek w moving average


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

        self.pi.set_mode(ENCODER_PIN_A, pigpio.INPUT)
        self.pi.set_pull_up_down(ENCODER_PIN_A, pigpio.PUD_UP)
        self.pi.set_glitch_filter(ENCODER_PIN_A, GLITCH_US)

        self.lock = threading.Lock()
        self.position_ticks = 0

        self.cb = self.pi.callback(
            ENCODER_PIN_A,
            pigpio.FALLING_EDGE,
            self.encoder_callback
        )

        self.prev_ticks = 0
        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.publish_state
        )

        self.ticks_per_rev = PPR
        self.wheel_circ = math.pi * WHEEL_DIAMETER

        # bufory moving average
        self.dt_buf = []
        self.tick_buf = []

    def dir_callback(self, msg):
        self.direction = int(msg.data)

    def encoder_callback(self, gpio, level, tick):
        if self.direction == 0:
            return
        with self.lock:
            self.position_ticks += self.direction

    def publish_state(self):

        now = self.get_clock().now()

        with self.lock:
            ticks = self.position_ticks

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        delta_ticks = ticks - self.prev_ticks

        # aktualizacja buforów moving average
        self.dt_buf.append(dt)
        self.tick_buf.append(delta_ticks)

        if len(self.dt_buf) > VEL_WINDOW:
            self.dt_buf.pop(0)
            self.tick_buf.pop(0)

        sum_dt = sum(self.dt_buf)
        sum_ticks = sum(self.tick_buf)

        # pozycja absolutna
        rev = ticks / self.ticks_per_rev
        angle = rev * 2.0 * math.pi
        distance = rev * self.wheel_circ

        # prędkość z moving average
        if sum_dt > 0:
            vel_rev = (sum_ticks / self.ticks_per_rev) / sum_dt
        else:
            vel_rev = 0.0

        omega = vel_rev * 2.0 * math.pi
        linear_vel = vel_rev * self.wheel_circ

        msg = Float64MultiArray()
        msg.data = [
            float(ticks),
            angle,
            distance,
            omega,
            linear_vel
        ]

        self.publisher.publish(msg)

        self.prev_ticks = ticks
        self.prev_time = now

    def destroy_node(self):
        self.cb.cancel()
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