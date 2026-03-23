import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from gpiozero import DigitalInputDevice
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
        
        self.position = 0   # stan pozycji (ticki kwadraturowe)

        self.prev_state = 0     # poprzedni stan AB
        
        self.prev_position = 0  # do estymacji prędkości
        self.prev_time = time.monotonic()

        self.encA = DigitalInputDevice(PIN_A, pull_up=True) # wejścia GPIO
        self.encB = DigitalInputDevice(PIN_B, pull_up=True)

        self.prev_state = (self.encA.value << 1) | self.encB.value  # inicjalny stan

        self.encA.when_activated = self.update      # callback na oba zbocza
        self.encA.when_deactivated = self.update
        self.encB.when_activated = self.update
        self.encB.when_deactivated = self.update

        self.timer = self.create_timer(0.001, self.publish_state)   # ~333 Hz publikacja stanu

        self.lookup = {       # tablica dekodera quadrature
            0b0001: +1,
            0b0010: -1,
            0b0100: -1,
            0b0111: +1,
            0b1000: +1,
            0b1011: -1,
            0b1101: -1,
            0b1110: +1,
        }

    def update(self):
        state = (self.encA.value << 1) | self.encB.value
        transition = (self.prev_state << 2) | state

        if transition in self.lookup:
            self.position += self.lookup[transition]

        self.prev_state = state

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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()