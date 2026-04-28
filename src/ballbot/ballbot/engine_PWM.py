#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Int32MultiArray
import pigpio

PIN_RPWM = 10
PIN_LPWM = 9
PIN_REN = 11
PIN_LEN = 5

PWM_FREQ = 20000
PWM_RANGE = 255

OMEGA_REF = 10.0   # stała wartość zadana do testów (rad/s)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class MotorNode(Node):

    def __init__(self):
        super().__init__('ZN_tuning_node')

        # --- pomiar prędkości z enkodera ---
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            'wheel_state',
            self.state_cb,
            10
        )

        # --- ręczna zmiana Kp ---
        self.kp_sub = self.create_subscription(
            Float32,
            'set_kp',
            self.kp_cb,
            10
        )

        self.dir_pub = self.create_publisher(
            Int32MultiArray,
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

        # --- regulator P ---
        self.kp = 1.0
        self.meas = 0.0

        self.log_counter = 0
        self.log_every = 10

        self.get_logger().info("ZN tuning node started")

    def kp_cb(self, msg):
        self.kp = float(msg.data)
        self.get_logger().info(f"NEW Kp = {self.kp}")

    def publish_dir(self, d):
        msg = Int32MultiArray()
        msg.data = [int(d), 0, 0]
        self.dir_pub.publish(msg)

    def apply_pwm(self, u):
        direction = 1 if u > 0 else -1 if u < 0 else 0
        duty = int(abs(u))

        if direction > 0:
            self.pi.set_PWM_dutycycle(PIN_LPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_RPWM, duty)
        elif direction < 0:
            self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_LPWM, duty)
        else:
            self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
            self.pi.set_PWM_dutycycle(PIN_LPWM, 0)

        self.publish_dir(direction)
        return duty, direction

    def state_cb(self, msg):
        if len(msg.data) < 5:
            return

        # zakładamy omega1 na indeksie 4
        self.meas = float(msg.data[4])

        # --- regulator P ---
        error = OMEGA_REF - self.meas
        u = self.kp * error

        # skalowanie do PWM
        u_pwm = clamp(u, -PWM_RANGE, PWM_RANGE)

        duty, direction = self.apply_pwm(u_pwm)

        # --- logowanie ---
        self.log_counter += 1
        if self.log_counter >= self.log_every:
            self.log_counter = 0
            self.get_logger().info(
                f"Kp={self.kp:.2f}, ref={OMEGA_REF:.2f}, meas={self.meas:.2f}, "
                f"err={error:.2f}, u={u_pwm:.1f}, pwm={duty}, dir={direction}"
            )

    def stop(self):
        self.pi.set_PWM_dutycycle(PIN_RPWM, 0)
        self.pi.set_PWM_dutycycle(PIN_LPWM, 0)
        self.publish_dir(0)


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