#!/usr/bin/env python3

import math  # funkcje matematyczne: abs, pi, ograniczenia PID
import rclpy  # type: ignore  # biblioteka ROS 2
from rclpy.node import Node  # type: ignore  # klasa bazowa dla noda ROS 2
from std_msgs.msg import Float64MultiArray, Int32MultiArray  # type: ignore  # wiadomości: prędkości i kierunek
import pigpio  # sterowanie GPIO/PWM na Raspberry Pi


WHEEL_COUNT = 3  # liczba kół / silników w układzie

# Indeksy prędkości kątowych w wiadomości wheel_state:
# [t, tick1, ang1, dist1, omega1, vel1, tick2, ang2, dist2, omega2, vel2, tick3, ang3, dist3, omega3, vel3]
WHEEL_STATE_ANGULAR_VEL_IDXS = [4, 9, 14]  # indeksy omega1, omega2, omega3 w tablicy wheel_state

# Piny sterowania silnikami:
# dla każdego silnika mamy RPWM, LPWM, REN, LEN
PIN_RPWM = [10, 4, 6]   # silnik 1, 2, 3: PWM dla kierunku dodatniego
PIN_LPWM = [9, 17, 13]  # silnik 1, 2, 3: PWM dla kierunku ujemnego
PIN_REN = [11, 8, 19]   # silnik 1, 2, 3: enable prawej gałęzi mostka
PIN_LEN = [5, 22, 26]   # silnik 1, 2, 3: enable lewej gałęzi mostka

PWM_FREQ = 20000  # częstotliwość PWM
PWM_RANGE = 255    # zakres PWM zgodny z pigpio


class PID:
    def __init__(
        self,
        kp: float,  # wzmocnienie proporcjonalne
        ki: float,  # wzmocnienie całkujące
        kd: float,  # wzmocnienie różniczkujące
        output_limit: float = 1.0,  # maksymalna wartość wyjścia PID
        i_limit: float = 0.5,  # ograniczenie członu całkującego
    ):
        self.kp = float(kp)  # zapis Kp jako float
        self.ki = float(ki)  # zapis Ki jako float
        self.kd = float(kd)  # zapis Kd jako float
        self.output_limit = float(output_limit)  # limit wyjścia regulatora
        self.i_limit = float(i_limit)  # limit całki, żeby nie narastała bez końca

        self.integral = 0.0  # człon całkujący błędu
        self.prev_error = 0.0  # poprzedni błąd do obliczenia pochodnej
        self.initialized = False  # flaga pierwszego wywołania regulatora

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        dt = max(float(dt), 1e-6)  # zabezpieczenie przed dzieleniem przez zero

        error = setpoint - measurement  # błąd regulacji: referencja minus pomiar

        if not self.initialized:  # pierwszy krok: nie mamy jeszcze poprzedniego błędu
            self.prev_error = error  # ustawiamy poprzedni błąd
            self.initialized = True  # regulator został zainicjalizowany

        self.integral += error * dt  # całkowanie błędu w czasie
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))  # ograniczenie wind-up

        derivative = (error - self.prev_error) / dt  # pochodna błędu
        self.prev_error = error  # zapis błędu do następnego kroku

        u = self.kp * error + self.ki * self.integral + self.kd * derivative  # wyjście PID
        u = max(-self.output_limit, min(self.output_limit, u))  # ograniczenie do zakresu sterowania
        return u  # sygnał sterujący: dodatni lub ujemny

    def reset(self):
        self.integral = 0.0  # zerowanie całki
        self.prev_error = 0.0  # zerowanie poprzedniego błędu
        self.initialized = False  # wymuszenie ponownej inicjalizacji


class WheelVelocityMotorNode(Node):
    def __init__(self):
        super().__init__('wheel_velocity_motor_node')  # nazwa noda ROS 2

        self.ref_sub = self.create_subscription(  # subskrypcja referencji prędkości
            Float64MultiArray,  # typ wiadomości z referencją dla 3 kół
            'vel_from_controller',  # topic z prędkościami zadanymi
            self.ref_callback,  # callback aktualizujący referencje
            10  # kolejka wiadomości
        )

        self.state_sub = self.create_subscription(  # subskrypcja bieżącego stanu kół
            Float64MultiArray,  # typ wiadomości wheel_state
            'wheel_state',  # topic z aktualnymi prędkościami i pozycjami
            self.state_callback,  # callback wykonujący regulację i sterowanie
            10  # kolejka wiadomości
        )

        self.dir_pub = self.create_publisher(  # publisher kierunku obrotu
            Int32MultiArray,  # tablica kierunków dla 3 silników
            'wheel_direction',  # topic z kierunkiem obrotu
            10  # kolejka wiadomości
        )

        self.ref_vel = [0.0, 0.0, 0.0]  # referencyjne prędkości kątowe dla 3 kół
        self.meas_vel = [0.0, 0.0, 0.0]  # zmierzone prędkości kątowe dla 3 kół

        self.last_time = self.get_clock().now()  # czas ostatniego przeliczenia PID

        self.pid = [  # trzy niezależne regulatory PID, po jednym na każde koło
            PID(kp=2.0, ki=0.4, kd=0.01, output_limit=1.0, i_limit=0.5),  # PID koła 1
            PID(kp=2.0, ki=0.4, kd=0.01, output_limit=1.0, i_limit=0.5),  # PID koła 2
            PID(kp=2.0, ki=0.4, kd=0.01, output_limit=1.0, i_limit=0.5),  # PID koła 3
        ]

        self.pi = pigpio.pi()  # połączenie z pigpio daemon
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        for i in range(WHEEL_COUNT):  # konfiguracja 3 silników
            self.pi.write(PIN_REN[i], 1)  # aktywacja gałęzi prawej
            self.pi.write(PIN_LEN[i], 1)  # aktywacja gałęzi lewej

            self.pi.set_PWM_frequency(PIN_RPWM[i], PWM_FREQ)  # częstotliwość PWM dla RPWM
            self.pi.set_PWM_frequency(PIN_LPWM[i], PWM_FREQ)  # częstotliwość PWM dla LPWM

            self.pi.set_PWM_range(PIN_RPWM[i], PWM_RANGE)  # zakres PWM dla RPWM
            self.pi.set_PWM_range(PIN_LPWM[i], PWM_RANGE)  # zakres PWM dla LPWM

        self.dir_msg = Int32MultiArray()  # wiadomość publikująca kierunek dla 3 silników
        self.dir_msg.data = [0, 0, 0]  # startowo zatrzymane

        self.stop_all()  # start od zatrzymania

        self.get_logger().info('Wheel velocity motor controller started')  # komunikat startowy

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) < WHEEL_COUNT:  # zabezpieczenie przed zbyt krótką wiadomością
            return  # ignorujemy błędne dane

        self.ref_vel[0] = float(msg.data[0])  # referencja prędkości dla koła 1
        self.ref_vel[1] = float(msg.data[1])  # referencja prędkości dla koła 2
        self.ref_vel[2] = float(msg.data[2])  # referencja prędkości dla koła 3

    def apply_motor(self, i: int, pwm_value: float, direction: int):
        duty = int(max(0.0, min(1.0, float(pwm_value))) * PWM_RANGE)  # PWM w zakresie 0..255

        if direction > 0:  # obrót do przodu
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0)  # wyłącz przeciwny kierunek
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], duty)  # ustaw PWM dla kierunku dodatniego

        elif direction < 0:  # obrót do tyłu
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0)  # wyłącz przeciwny kierunek
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], duty)  # ustaw PWM dla kierunku ujemnego

        else:  # stop
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0)  # wyłącz PWM dodatni
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0)  # wyłącz PWM ujemny

    def stop_all(self):
        for i in range(WHEEL_COUNT):  # zatrzymanie wszystkich silników
            self.pi.set_PWM_dutycycle(PIN_RPWM[i], 0)  # PWM dodatni = 0
            self.pi.set_PWM_dutycycle(PIN_LPWM[i], 0)  # PWM ujemny = 0
            self.dir_msg.data[i] = 0  # kierunek = 0
        self.dir_pub.publish(self.dir_msg)  # publikacja stanu stop

    def state_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 16:  # wheel_state musi zawierać pełne dane dla 3 kół
            return  # ignorujemy błędną wiadomość

        # Bierzemy prędkości kątowe z wheel_state.
        self.meas_vel[0] = float(msg.data[WHEEL_STATE_ANGULAR_VEL_IDXS[0]])  # omega koła 1
        self.meas_vel[1] = float(msg.data[WHEEL_STATE_ANGULAR_VEL_IDXS[1]])  # omega koła 2
        self.meas_vel[2] = float(msg.data[WHEEL_STATE_ANGULAR_VEL_IDXS[2]])  # omega koła 3

        now = self.get_clock().now()  # aktualny czas systemowy ROS
        dt = (now - self.last_time).nanoseconds * 1e-9  # czas od ostatniej aktualizacji PID
        self.last_time = now  # zapis czasu do kolejnej iteracji

        if dt <= 0.0:  # zabezpieczenie gdy czas nie poszedł do przodu
            dt = 0.001  # minimalny krok czasowy

        pwm_out = [0.0, 0.0, 0.0]  # wyjściowe wartości PWM dla 3 silników
        dir_out = [0, 0, 0]  # wyjściowy kierunek: -1, 0, 1 dla 3 silników

        for i in range(WHEEL_COUNT):  # osobna regulacja dla każdego koła
            u = self.pid[i].update(self.ref_vel[i], self.meas_vel[i], dt)  # sygnał PID dla i-tego koła

            # PWM jako wartość bezwzględna, a kierunek osobno.
            if abs(u) < 1e-4:  # martwa strefa dla bardzo małych sygnałów
                dir_out[i] = 0  # brak ruchu
                pwm_out[i] = 0.0  # brak PWM
            elif u > 0.0:  # dodatni sygnał -> obrót w przód / zgodny kierunek
                dir_out[i] = 1  # kierunek dodatni
                pwm_out[i] = u  # PWM równe wartości sterującej
            else:  # u < 0, czyli obrót w przeciwną stronę
                dir_out[i] = -1  # kierunek ujemny
                pwm_out[i] = -u  # PWM jako wartość dodatnia

            self.apply_motor(i, pwm_out[i], dir_out[i])  # fizyczne ustawienie PWM na silniku

        self.dir_msg.data = dir_out  # zapis kierunku dla wszystkich 3 kół do jednej wiadomości
        self.dir_pub.publish(self.dir_msg)  # publikacja kierunku obrotu

    def destroy_node(self):
        self.stop_all()  # zatrzymanie silników przed wyjściem
        self.pi.stop()  # zamknięcie połączenia z pigpio
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)  # inicjalizacja ROS 2
    node = WheelVelocityMotorNode()  # utworzenie jednego noda sterującego prędkością i silnikami

    try:
        rclpy.spin(node)  # główna pętla zdarzeń ROS 2
    except KeyboardInterrupt:
        pass  # normalne zakończenie po Ctrl+C
    finally:
        node.destroy_node()  # zwolnienie zasobów noda
        rclpy.shutdown()  # zamknięcie ROS 2


if __name__ == '__main__':
    main()  # punkt wejścia programu