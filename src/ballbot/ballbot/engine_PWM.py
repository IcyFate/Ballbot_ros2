import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio


PIN_RPWM = 4
PIN_LPWM = 19
PIN_REN = 3
PIN_LEN = 2

PWM_FREQ = 20000
PWM_RANGE = 255


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        self.sub = self.create_subscription(
            Float32,
            'motor_cmd',
            self.cmd_cb,
            10
        )

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        # enable mostka
        self.pi.write(PIN_REN, 1)
        self.pi.write(PIN_LEN, 1)

        # konfiguracja PWM
        self.pi.set_PWM_frequency(PIN_RPWM, PWM_FREQ)
        self.pi.set_PWM_frequency(PIN_LPWM, PWM_FREQ)

        self.pi.set_PWM_range(PIN_RPWM, PWM_RANGE)
        self.pi.set_PWM_range(PIN_LPWM, PWM_RANGE)

        self.stop()

    def stop(self):
        self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
        self.pi.set_PWM_dutycycle(PIN_LPWM, 0)

    def cmd_cb(self, msg):
        u = max(-1.0, min(1.0, msg.data))
        duty = int(abs(u) * PWM_RANGE)

        if u > 0:
            self.pi.set_PWM_dutycycle(PIN_LPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_RPWM, duty)

        elif u < 0:
            self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_LPWM, duty)

        else:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop()
    node.pi.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()