import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import pigpio
import time


PIN_A = 24
PIN_B = 23


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')

        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            'encoder_state',
            10
        )

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        self.pi.set_mode(PIN_A, pigpio.INPUT)
        self.pi.set_mode(PIN_B, pigpio.INPUT)
        self.pi.set_pull_up_down(PIN_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(PIN_B, pigpio.PUD_UP)

        self.position = 0

        self.prev_position = 0
        self.prev_time = time.monotonic()

        # callback osobno dla A i B
        self.cbA = self.pi.callback(PIN_A, pigpio.EITHER_EDGE, self.edge_A)
        self.cbB = self.pi.callback(PIN_B, pigpio.EITHER_EDGE, self.edge_B)

        self.timer = self.create_timer(0.0005, self.publish_state)

    def edge_A(self, gpio, level, tick):
        b = self.pi.read(PIN_B)

        # klasyczna reguła quadrature
        if level == b:
            self.position += 1
        else:
            self.position -= 1

    def edge_B(self, gpio, level, tick):
        a = self.pi.read(PIN_A)

        if level != a:
            self.position += 1
        else:
            self.position -= 1

    def publish_state(self):
        now = time.monotonic()
        dt = now - self.prev_time

        pos = self.position
        vel = (pos - self.prev_position) / dt

        self.prev_position = pos
        self.prev_time = now

        msg = Int32MultiArray()
        msg.data = [int(pos), int(vel)]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cbA.cancel()
    node.cbB.cancel()
    node.pi.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()