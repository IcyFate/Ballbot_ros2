import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import pigpio


PIN_RPWM = 4                # silnik 1: 10      ?silnik 2: 4     silnik 3: 6
PIN_LPWM = 17               # silnik 1: 9       silnik 2: 17     silnik 3: 13
PIN_REN = 8                 # silnik 1: 11      silnik 2: 8     silnik 3: 19
PIN_LEN = 22                # silnik 1: 5       silnik 2: 22     silnik 3: 26

PWM_FREQ = 20000
PWM_RANGE = 255


class MotorNode(Node):

    def __init__(self):
        super().__init__('DC_PWM')

        self.sub = self.create_subscription(
            Float32,
            'motor_cmd',
            self.cmd_cb,
            10
        )

        self.dir_pub = self.create_publisher(
            Int32,
            'motor_direction',
            10
        )

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        self.pi.write(PIN_REN, 1)
        self.pi.write(PIN_LEN, 1)

        self.pi.set_PWM_frequency(PIN_RPWM, PWM_FREQ)
        self.pi.set_PWM_frequency(PIN_LPWM, PWM_FREQ)

        self.pi.set_PWM_range(PIN_RPWM, PWM_RANGE)
        self.pi.set_PWM_range(PIN_LPWM, PWM_RANGE)

        self.stop()

    def publish_dir(self, d):
        msg = Int32()
        msg.data = d
        self.dir_pub.publish(msg)

    def stop(self):
        self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
        self.pi.set_PWM_dutycycle(PIN_LPWM, 0)
        self.publish_dir(0)

    def cmd_cb(self, msg):
        u = max(-1.0, min(1.0, msg.data))
        duty = int(abs(u) * PWM_RANGE)

        if u > 0:
            self.pi.set_PWM_dutycycle(PIN_LPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_RPWM, duty)
            self.publish_dir(1)

        elif u < 0:
            self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_LPWM, duty)
            self.publish_dir(-1)

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