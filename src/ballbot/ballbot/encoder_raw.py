import rclpy   # pyright: ignore[reportMissingImports]
from rclpy.node import Node   # pyright: ignore[reportMissingImports]
from std_msgs.msg import Int32MultiArray   # pyright: ignore[reportMissingImports]

import pigpio

ENCODER_PIN_B = 23
ENCODER_PIN_A = 24


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')

        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            'encoder_ticks',
            10
        )

        self.count_A = 0
        self.count_B = 0

        # --- pigpio init ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running")

        # wejścia + pull-up
        self.pi.set_mode(ENCODER_PIN_A, pigpio.INPUT)
        self.pi.set_mode(ENCODER_PIN_B, pigpio.INPUT)

        self.pi.set_pull_up_down(ENCODER_PIN_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(ENCODER_PIN_B, pigpio.PUD_UP)

        # callback na zbocze opadające (odpowiednik when_pressed)
        self.cb_A = self.pi.callback(
            ENCODER_PIN_A,
            pigpio.FALLING_EDGE,
            self.encoder_callback_A
        )

        self.cb_B = self.pi.callback(
            ENCODER_PIN_B,
            pigpio.FALLING_EDGE,
            self.encoder_callback_B
        )

        self.timer = self.create_timer(0.001, self.publish_ticks)

    def encoder_callback_A(self, gpio, level, tick):
        self.count_A += 1

    def encoder_callback_B(self, gpio, level, tick):
        self.count_B += 1

    def publish_ticks(self):
        msg = Int32MultiArray()
        msg.data = [self.count_A, self.count_B]
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cb_A.cancel()
        self.cb_B.cancel()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()