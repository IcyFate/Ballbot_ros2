import rclpy    # pyright: ignore[reportMissingImports]
from rclpy.node import Node # pyright: ignore[reportMissingImports]
from std_msgs.msg import Int32  # pyright: ignore[reportMissingImports]

from gpiozero import Button

ENCODER_PIN = 23   # numer BCM


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')

        self.publisher_ = self.create_publisher(Int32, 'encoder_ticks', 10)

        self.count = 0

        self.encoder_button = Button(ENCODER_PIN, pull_up=True)
        self.encoder_button.when_pressed = self.encoder_callback

        self.timer = self.create_timer(0.01, self.publish_ticks)

    def encoder_callback(self, channel):
        self.count += 1

    def publish_ticks(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)


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