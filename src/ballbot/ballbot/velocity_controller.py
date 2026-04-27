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

        return p + self.ki * self.integral           # wyjście PI

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
            PI(kp=5.0, ki=5.0, i_limit=120.0),       # PI koła 1
            PI(kp=5.0, ki=5.0, i_limit=120.0),       # PI koła 2
            PI(kp=5.0, ki=5.0, i_limit=120.0),       # PI koła 3
        ]

        self.pi = pigpio.pi()                         # połączenie z daemonem pigpio
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running") # brak daemonu = brak sterowania

        for i in range(WHEEL_COUNT):
            self.pi.write(PIN_REN[i], 1)              # enable prawej gałęzi mostka
            self.pi.write(PIN_LEN[i], 1)              # enable lewej gałęzi mostka
            self.pi.set_PWM_frequency(PIN_RPWM[i], PWM_FREQ)  # częstotliwość PWM
            self.pi.set_PWM_frequency(PIN_LPWM[i], PWM_FREQ)  # częstotliwość PWM
            self.pi.set_PWM_range(PIN_RPWM[i], PWM_RANGE)     # zakres PWM
            self.pi.set_PWM_range(PIN_LPWM[i], PWM_RANGE)     # zakres PWM

        self.dir_msg = Int32MultiArray()              # wiadomość o kierunku
        self.dir_msg.data = [0, 0, 0]                 # start od zatrzymania

        self.log_counter = 0                          # licznik sterowań do logowania
        self.log_every = 25                           # loguj co 25 iteracji

        self.stop_all()                               # bezpieczny start
        self.get_logger().info("Wheel velocity PI controller started")

    def ref_callback(self, msg):
        if len(msg.data) < WHEEL_COUNT:
            return

        for i in range(WHEEL_COUNT):
            self.ref_vel[i] = float(msg.data[i])      # zapis zadanej prędkości

    def speed_to_pwm_ff(self, omega):
        omega = clamp(abs(omega), 0.0, OMEGA_MAX)     # ograniczenie zakresu
        if omega == 0.0:
            return 0.0                                # brak zadania = brak feed-forward
        return PWM_START_MOVE + (PWM_MAX - PWM_START_MOVE) * (omega / OMEGA_MAX)  # baza PWM

    def apply_motor(self, i, duty, direction):
        duty = int(clamp(duty, 0.0, PWM_MAX))         # konwersja na dutycycle 0..255

        if direction > 0:
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0) # wyłączenie przeciwnego kierunku
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], duty)  # PWM dodatni
        elif direction < 0:
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0) # wyłączenie przeciwnego kierunku
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], duty)  # PWM ujemny
        else:
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0) # oba PWM = 0
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0) # oba PWM = 0

        return duty                                   # zwrot realnie ustawionego PWM

    def stop_all(self):
        for i in range(WHEEL_COUNT):
            self.apply_motor(i, 0, 0)                # zatrzymanie silnika
            self.pi_ctrl[i].reset()                  # reset PI
            self.last_direction[i] = 0               # reset kierunku

        self.dir_msg.data = [0, 0, 0]                # publikacja stop
        self.dir_pub.publish(self.dir_msg)

    def state_callback(self, msg):
        if len(msg.data) < 16:
            return

        for i in range(WHEEL_COUNT):
            self.meas_vel[i] = float(msg.data[WHEEL_STATE_ANGULAR_VEL_IDXS[i]])  # pomiar prędkości

        now = self.get_clock().now()                  # aktualny czas
        dt = (now - self.last_time).nanoseconds * 1e-9  # czas próbkowania
        self.last_time = now                          # zapis czasu
        dt = clamp(dt, DT_MIN, DT_MAX)                # bezpieczny zakres dt

        dir_out = [0, 0, 0]                           # kierunek wyjściowy
        pwm_out = [0.0, 0.0, 0.0]                     # PWM po regulacji

        for i in range(WHEEL_COUNT):
            ref = self.ref_vel[i]                     # zadana prędkość
            meas = self.meas_vel[i]                   # zmierzona prędkość

            if abs(ref) < REF_DEADBAND_OMEGA:
                self.pi_ctrl[i].reset()               # reset PI przy zatrzymaniu
                self.apply_motor(i, 0, 0)             # pełny stop
                dir_out[i] = 0
                pwm_out[i] = 0.0
                continue

            direction = 1 if ref > 0.0 else -1        # kierunek z znaku zadania

            if direction != self.last_direction[i] and self.last_direction[i] != 0:
                self.pi_ctrl[i].reset()               # reset przy zmianie kierunku

            self.last_direction[i] = direction        # zapamiętanie aktualnego kierunku

            ref_abs = abs(ref)                        # regulacja modułu prędkości
            meas_abs = abs(meas)                      # moduł prędkości z pomiaru

            pwm_ff = self.speed_to_pwm_ff(ref_abs)    # feed-forward z prędkości zadanej
            pwm_corr = self.pi_ctrl[i].update(ref_abs, meas_abs, dt)  # korekta PI

            duty = pwm_ff + pwm_corr                  # suma sterowania
            duty = clamp(duty, 0.0, PWM_MAX)          # ograniczenie do fizycznego PWM

            if duty > 0.0:
                duty = max(PWM_START_MOVE, duty)      # minimum PWM tylko dla ruchu

            pwm_out[i] = duty                         # zapis do logów
            dir_out[i] = direction                    # zapis kierunku
            self.apply_motor(i, duty, direction)      # fizyczne ustawienie silnika

        self.dir_msg.data = dir_out                   # publikacja kierunku
        self.dir_pub.publish(self.dir_msg)

        self.log_counter += 1                         # licznik sterowań
        if self.log_counter >= self.log_every:
            self.log_counter = 0
            self.get_logger().info(
                f"ref={self.ref_vel}, meas={self.meas_vel}, "
                f"pwm={['{:.1f}'.format(v) for v in pwm_out]}, "
                f"dir={dir_out}"
            )                                         # log co 25 sterowań

    def destroy_node(self):
        self.stop_all()                               # bezpieczne wyłączenie silników
        self.pi.stop()                                # zamknięcie pigpio
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)                             # inicjalizacja ROS 2
    node = WheelVelocityMotorNode()                   # utworzenie noda

    try:
        rclpy.spin(node)                              # pętla zdarzeń
    except KeyboardInterrupt:
        pass                                          # normalne przerwanie
    finally:
        node.destroy_node()                           # sprzątanie zasobów
        rclpy.shutdown()                              # zamknięcie ROS 2


if __name__ == "__main__":
    main()                                            # punkt wejścia