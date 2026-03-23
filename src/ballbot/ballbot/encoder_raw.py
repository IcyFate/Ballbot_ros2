import rclpy    # pyright: ignore[reportMissingImports]
from rclpy.node import Node # pyright: ignore[reportMissingImports]
from std_msgs.msg import Int32  # pyright: ignore[reportMissingImports]
from std_msgs.msg import Int32MultiArray

from gpiozero import Button

ENCODER_PIN_B = 23   # pin B of the encoder
ENCODER_PIN_A = 24   # pin A of the encoder


class EncoderNode(Node):

    def __init__(self):
        super().__init__('encoder_node')

        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)

        self.count_A = 0
        self.count_B = 0

        self.encoder_button_B = Button(ENCODER_PIN_B, pull_up=True)
        self.encoder_button_B.when_pressed = self.encoder_callback_B

        self.encoder_button_A = Button(ENCODER_PIN_A, pull_up=True)
        self.encoder_button_A.when_pressed = self.encoder_callback_A

        self.timer = self.create_timer(0.001, self.publish_ticks)

    def encoder_callback_A(self):
        self.count_A += 1

    def encoder_callback_B(self):
        self.count_B += 1

    def publish_ticks(self):
        msg = Int32MultiArray()
        msg.data = [self.count_A, self.count_B]
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