#!/usr/bin/env python3
from __future__ import annotations

import math
import pigpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray

WHEEL_COUNT = 3                                      # liczba kół

WHEEL_STATE_ANGULAR_VEL_IDXS = [4, 9, 14]            # indeksy omega1..omega3 w wheel_state

PIN_RPWM = [10, 4, 6]
PIN_LPWM = [9, 17, 13]
PIN_REN = [11, 8, 19]
PIN_LEN = [5, 22, 26]

PWM_FREQ = 20000
PWM_RANGE = 255

OMEGA_MAX = 330.0 * 2.0 * math.pi / 60.0             # max rad/s
PWM_START_MOVE = 25.0                                # PWM potrzebny do ruszenia
PWM_MAX = 255.0

REF_DEADBAND_OMEGA = 0.05                            # martwa strefa wokół zera
DT_MIN = 1e-3
DT_MAX = 0.05


def clamp(x, lo, hi):
    return max(lo, min(hi, x))                       # ograniczenie zakresu


class PI:
    def __init__(self, kp, ki, i_limit=120.0):
        self.kp = float(kp)                          # wzmocnienie P
        self.ki = float(ki)                          # wzmocnienie I
        self.i_limit = float(i_limit)                # limit całki

        self.integral = 0.0                          # stan całki
        self.initialized = False                     # flaga pierwszego kroku

    def update(self, setpoint, measurement, dt):
        dt = clamp(float(dt), DT_MIN, DT_MAX)       # stabilny krok czasowy

        error = setpoint - measurement               # błąd regulacji e = r - y
        if not self.initialized:
            self.initialized = True                  # inicjalizacja regulatora

        p = self.kp * error                          # człon proporcjonalny

        i_candidate = self.integral + error * dt     # całkowanie błędu
        i_candidate = clamp(i_candidate, -self.i_limit, self.i_limit)  # ograniczenie całki

        u_unsat = p + self.ki * i_candidate          # sygnał przed saturacją

        # Anti-windup przez warunkową akceptację całki
        if u_unsat >= PWM_MAX and error > 0.0:
            pass                                     # nie zwiększaj całki przy dodatnim nasyceniu
        elif u_unsat <= 0.0 and error < 0.0:
            pass                                     # nie zwiększaj całki przy dolnym nasyceniu
        else:
            self.integral = i_candidate              # akceptacja całki, gdy nie pogarsza nasycenia

        return p + self.ki * self.integral, error    # zwracamy też error do logowania

    def reset(self):
        self.integral = 0.0                          # wyzerowanie całki
        self.initialized = False                     # reset stanu


class WheelVelocityMotorNode(Node):
    def __init__(self):
        super().__init__("wheel_velocity_motor_node")  # nazwa noda

        self.ref_sub = self.create_subscription(
            Float64MultiArray,
            "vel_from_controller",
            self.ref_callback,
            10,
        )                                             # referencje prędkości

        self.state_sub = self.create_subscription(
            Float64MultiArray,
            "wheel_state",
            self.state_callback,
            10,
        )                                             # stan kół / prędkości

        self.dir_pub = self.create_publisher(
            Int32MultiArray,
            "motor_direction",
            10,
        )                                             # publikacja kierunku silników

        self.ref_vel = [0.0, 0.0, 0.0]                # zadane prędkości
        self.meas_vel = [0.0, 0.0, 0.0]               # zmierzone prędkości
        self.last_time = self.get_clock().now()       # czas poprzedniej iteracji
        self.last_direction = [0, 0, 0]               # pamięć kierunku do resetu PI

        self.pi_ctrl = [
            PI(kp=10, ki=5, i_limit=120.0),
            PI(kp=10, ki=5, i_limit=120.0),
            PI(kp=10, ki=5, i_limit=120.0),
        ]

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        for i in range(WHEEL_COUNT):
            self.pi.write(PIN_REN[i], 1)
            self.pi.write(PIN_LEN[i], 1)
            self.pi.set_PWM_frequency(PIN_RPWM[i], PWM_FREQ)
            self.pi.set_PWM_frequency(PIN_LPWM[i], PWM_FREQ)
            self.pi.set_PWM_range(PIN_RPWM[i], PWM_RANGE)
            self.pi.set_PWM_range(PIN_LPWM[i], PWM_RANGE)

        self.dir_msg = Int32MultiArray()
        self.dir_msg.data = [0, 0, 0]

        self.log_counter = 0
        self.log_every = 25

        self.stop_all()
        self.get_logger().info("Wheel velocity PI controller started")

    def ref_callback(self, msg):
        if len(msg.data) < WHEEL_COUNT:
            return

        for i in range(WHEEL_COUNT):
            self.ref_vel[i] = float(msg.data[i])

    def apply_motor(self, i, duty, direction):
        duty = int(clamp(duty, 0.0, PWM_MAX))

        if direction > 0:
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0)
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], duty)
        elif direction < 0:
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0)
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], duty)
        else:
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0)
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0)

        return duty

    def stop_all(self):
        for i in range(WHEEL_COUNT):
            self.apply_motor(i, 0, 0)
            self.pi_ctrl[i].reset()
            self.last_direction[i] = 0

        self.dir_msg.data = [0, 0, 0]
        self.dir_pub.publish(self.dir_msg)

    def state_callback(self, msg):
        if len(msg.data) < 16:
            return

        for i in range(WHEEL_COUNT):
            self.meas_vel[i] = float(msg.data[WHEEL_STATE_ANGULAR_VEL_IDXS[i]])

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        dt = clamp(dt, DT_MIN, DT_MAX)

        dir_out = [0, 0, 0]
        pwm_out = [0.0, 0.0, 0.0]
        err_out = [0.0, 0.0, 0.0]
        int_out = [0.0, 0.0, 0.0]

        for i in range(WHEEL_COUNT):
            ref = self.ref_vel[i]
            meas = self.meas_vel[i]

            if abs(ref) < REF_DEADBAND_OMEGA:
                self.pi_ctrl[i].reset()
                self.apply_motor(i, 0, 0)
                dir_out[i] = 0
                pwm_out[i] = 0.0
                continue

            direction = 1 if ref > 0.0 else -1

            if direction != self.last_direction[i] and self.last_direction[i] != 0:
                self.pi_ctrl[i].reset()

            self.last_direction[i] = direction

            ref_abs = abs(ref)
            meas_abs = abs(meas)

            # CZYSTY PI – brak feed-forward
            u, error = self.pi_ctrl[i].update(ref_abs, meas_abs, dt)

            duty = clamp(u, 0.0, PWM_MAX)

            if duty > 0.0:
                duty = max(PWM_START_MOVE, duty)

            pwm_out[i] = duty
            dir_out[i] = direction
            err_out[i] = error
            int_out[i] = self.pi_ctrl[i].integral

            self.apply_motor(i, duty, direction)

        self.dir_msg.data = dir_out
        self.dir_pub.publish(self.dir_msg)

        self.log_counter += 1
        if self.log_counter >= self.log_every:
            self.log_counter = 0
            self.get_logger().info(
                f"ref={self.ref_vel}, meas={self.meas_vel}, "
                f"err={['{:.2f}'.format(e) for e in err_out]}, "
                f"I={['{:.2f}'.format(i) for i in int_out]}, "
                f"u={['{:.1f}'.format(v) for v in pwm_out]}, "
                f"dir={dir_out}"
            )

    def destroy_node(self):
        self.stop_all()
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelocityMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()