#!/usr/bin/env python3
import sys
import threading
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import pigpio

PIN_RPWM = 10
PIN_LPWM = 9
PIN_REN = 11
PIN_LEN = 5

PWM_FREQ = 20000
PWM_RANGE = 255

REF_MAG_INIT = 20.0
REF_SWITCH_PERIOD = 4.0

KP_STEP = 0.5
KI_STEP = 0.5
KD_STEP = 0.1
REF_STEP = 1.0

DT_MIN = 1e-3
DT_MAX = 0.05
PWM_MAX = 255.0

REF_DEADBAND_OMEGA = 0.05
PWM_START_MOVE = 25.0

D_FILTER_TAU = 0.03


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PID:
    def __init__(self, kp, ki, kd, i_limit=120.0, d_tau=0.03):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_limit = float(i_limit)
        self.d_tau = float(d_tau)

        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filtered = 0.0
        self.initialized = False

    def update(self, setpoint, measurement, dt):
        dt = clamp(float(dt), DT_MIN, DT_MAX)

        error = setpoint - measurement

        if not self.initialized:
            self.initialized = True
            self.prev_error = error
            self.d_filtered = 0.0

        p = self.kp * error

        i_candidate = self.integral + error * dt
        i_candidate = clamp(i_candidate, -self.i_limit, self.i_limit)

        derivative = (error - self.prev_error) / dt
        alpha = dt / (self.d_tau + dt)
        self.d_filtered = self.d_filtered + alpha * (derivative - self.d_filtered)

        u_unsat = p + self.ki * i_candidate + self.kd * self.d_filtered

        if u_unsat >= PWM_MAX and error > 0.0:
            pass
        elif u_unsat <= 0.0 and error < 0.0:
            pass
        else:
            self.integral = i_candidate

        self.prev_error = error

        return p + self.ki * self.integral + self.kd * self.d_filtered, error

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_filtered = 0.0
        self.initialized = False


class MotorNode(Node):
    def __init__(self):
        super().__init__('ZN_tuning_node')

        self.state_sub = self.create_subscription(
            Float64MultiArray,
            'wheel_state',
            self.state_cb,
            10
        )

        self.dir_pub = self.create_publisher(
            Int32MultiArray,
            'motor_direction',
            10
        )

        self.pi_hw = pigpio.pi()
        if not self.pi_hw.connected:
            raise RuntimeError("pigpiod not running")

        self.pi_hw.write(PIN_REN, 1)
        self.pi_hw.write(PIN_LEN, 1)

        self.pi_hw.set_PWM_frequency(PIN_RPWM, PWM_FREQ)
        self.pi_hw.set_PWM_frequency(PIN_LPWM, PWM_FREQ)

        self.pi_hw.set_PWM_range(PIN_RPWM, PWM_RANGE)
        self.pi_hw.set_PWM_range(PIN_LPWM, PWM_RANGE)

        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        self.pid_ctrl = PID(self.kp, self.ki, self.kd, i_limit=120.0, d_tau=D_FILTER_TAU)

        self.meas = 0.0
        self.last_time = self.get_clock().now()

        self.ref_mag = REF_MAG_INIT
        self.omega_ref = self.ref_mag
        self.ref_sign = 1
        self.last_direction = 0

        self.log_counter = 0
        self.log_every = 10

        self._stop_event = threading.Event()
        self._keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.ref_timer = self.create_timer(REF_SWITCH_PERIOD, self.switch_reference)

        self.get_logger().info("ZN tuning PID node started")
        self.get_logger().info(
            f"Start: Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}, ref={self.omega_ref:.2f} rad/s"
        )
        self.get_logger().info(
            "Keys: q/a -> Kp +/-0.5, w/s -> Ki +/-0.5, e/d -> Kd +/-0.1, t/g -> ref +/-1.0, x -> exit"
        )

    def print_status(self):
        self.get_logger().info(
            f"ACTUAL: Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}, "
            f"ref_mag={self.ref_mag:.2f}, ref={self.omega_ref:.2f} rad/s"
        )

    def update_reference_from_mag(self):
        if self.ref_mag < REF_DEADBAND_OMEGA:
            self.omega_ref = 0.0
        else:
            self.omega_ref = self.ref_sign * self.ref_mag

    def switch_reference(self):
        self.ref_sign *= -1
        self.update_reference_from_mag()

        if self.last_direction != 0:
            self.pid_ctrl.reset()
            self.last_direction = 0

        self.get_logger().info(f"NEW reference = {self.omega_ref:.2f} rad/s")
        self.print_status()

    def keyboard_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not self._stop_event.is_set():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)

                if ch == 'q':
                    self.kp += KP_STEP
                elif ch == 'a':
                    self.kp = max(0.0, self.kp - KP_STEP)
                elif ch == 'w':
                    self.ki += KI_STEP
                elif ch == 's':
                    self.ki = max(0.0, self.ki - KI_STEP)
                elif ch == 'e':
                    self.kd += KD_STEP
                elif ch == 'd':
                    self.kd = max(0.0, self.kd - KD_STEP)
                elif ch == 't':
                    self.ref_mag += REF_STEP
                    self.update_reference_from_mag()
                elif ch == 'g':
                    self.ref_mag = max(0.0, self.ref_mag - REF_STEP)
                    self.update_reference_from_mag()
                elif ch == 'x':
                    self.get_logger().info("Exit requested from keyboard")
                    self._stop_event.set()
                    rclpy.shutdown()
                    break
                else:
                    continue

                self.pid_ctrl.kp = self.kp
                self.pid_ctrl.ki = self.ki
                self.pid_ctrl.kd = self.kd
                self.print_status()

        except Exception as e:
            self.get_logger().error(f"Keyboard thread error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def publish_dir(self, d):
        msg = Int32MultiArray()
        msg.data = [int(d), 0, 0]
        self.dir_pub.publish(msg)

    def apply_pwm(self, duty, direction):
        duty = int(clamp(duty, 0.0, PWM_MAX))

        if direction > 0:
            self.pi_hw.set_PWM_dutycycle(PIN_LPWM, 0)
            self.pi_hw.set_PWM_dutycycle(PIN_RPWM, duty)
        elif direction < 0:
            self.pi_hw.set_PWM_dutycycle(PIN_RPWM, 0)
            self.pi_hw.set_PWM_dutycycle(PIN_LPWM, duty)
        else:
            self.pi_hw.set_PWM_dutycycle(PIN_RPWM, 0)
            self.pi_hw.set_PWM_dutycycle(PIN_LPWM, 0)

        self.publish_dir(direction)
        return duty

    def state_cb(self, msg):
        if len(msg.data) < 5:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        dt = clamp(dt, DT_MIN, DT_MAX)

        self.meas = float(msg.data[4])

        if abs(self.omega_ref) < REF_DEADBAND_OMEGA:
            self.pid_ctrl.reset()
            self.last_direction = 0
            self.apply_pwm(0, 0)
            return

        direction = 1 if self.omega_ref > 0.0 else -1

        if direction != self.last_direction and self.last_direction != 0:
            self.pid_ctrl.reset()

        self.last_direction = direction

        ref_abs = abs(self.omega_ref)
        meas_abs = abs(self.meas)

        u, error = self.pid_ctrl.update(ref_abs, meas_abs, dt)

        duty = clamp(u, 0.0, PWM_MAX)
        if duty > 0.0:
            duty = max(PWM_START_MOVE, duty)

        duty = self.apply_pwm(duty, direction)

        self.log_counter += 1
        if self.log_counter >= self.log_every:
            self.log_counter = 0
            self.get_logger().info(
                f"Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}, "
                f"ref_mag={self.ref_mag:.2f}, ref={self.omega_ref:.2f}, meas={self.meas:.2f}, "
                f"err={error:.2f}, I={self.pid_ctrl.integral:.2f}, "
                f"D={self.pid_ctrl.d_filtered:.2f}, u={duty:.1f}, dir={direction}"
            )

    def stop(self):
        self.pid_ctrl.reset()
        self.pi_hw.set_PWM_dutycycle(PIN_RPWM, 0)
        self.pi_hw.set_PWM_dutycycle(PIN_LPWM, 0)
        self.publish_dir(0)

    def destroy_node(self):
        self._stop_event.set()
        self.stop()
        self.pi_hw.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()