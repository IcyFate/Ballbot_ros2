#!/usr/bin/env python3
from __future__ import annotations  # pozwala używać adnotacji typów, które odnoszą się do klas zdefiniowanych później

import math  # funkcje matematyczne: sin, cos, atan2, radians
from typing import Optional  # typ opcjonalny, używany do oznaczenia braku danych na początku pracy noda

import numpy as np  # obliczenia macierzowe potrzebne do modelu stanu, Kalmana i LQR
import rclpy  # biblioteka ROS2 dla Pythona
from rclpy.node import Node  # klasa bazowa dla wszystkich nodów ROS2
from std_msgs.msg import Float64MultiArray  # wiadomość z tablicą floatów, używana do IMU, enkoderów i komend


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))  # ogranicza wartość x do przedziału [lo, hi]


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))  # zawija kąt do zakresu [-pi, pi], żeby uniknąć skoków 2*pi


def euler_discretize(A: np.ndarray, B: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    n = A.shape[0]  # liczba stanów modelu
    Ad = np.eye(n) + A * dt  # dyskretyzacja metodą Eulera: A_d = I + A*T_s
    Bd = B * dt  # dyskretyzacja wejścia: B_d = B*T_s
    return Ad, Bd  # zwracamy dyskretne macierze modelu


def solve_dare_iterative(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    max_iter: int = 1000,
    tol: float = 1e-10,
) -> np.ndarray:
    P = Q.copy()  # startowa macierz Riccatiego; inicjalizacja od Q jest typowa i praktyczna
    for _ in range(max_iter):  # iteracyjnie rozwiązujemy dyskretne równanie Riccatiego
        BT_P = B.T @ P  # pomocniczy iloczyn B^T P
        S = R + BT_P @ B  # macierz pośrednia: R + B^T P B
        try:
            S_inv = np.linalg.inv(S)  # próbujemy normalnej odwrotności
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)  # jeśli S jest osobliwa, używamy pseudoodwrotności

        P_next = A.T @ P @ A - A.T @ P @ B @ S_inv @ BT_P @ A + Q  # jedna iteracja równania Riccatiego
        if np.max(np.abs(P_next - P)) < tol:  # warunek zbieżności na podstawie maksymalnej zmiany elementu
            return P_next  # zwracamy ustabilizowane rozwiązanie
        P = P_next  # przechodzimy do kolejnej iteracji
    return P  # zwrócenie ostatniego przybliżenia, jeśli nie osiągnięto tolerancji


def solve_kalman_gain_iterative(
    A: np.ndarray,
    C: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    max_iter: int = 1000,
    tol: float = 1e-10,
) -> np.ndarray:
    P = Q.copy()  # inicjalizacja kowariancji estymatora
    for _ in range(max_iter):  # iteracyjne rozwiązanie równania Riccatiego dla filtru Kalmana
        CP = C @ P  # pomocniczy iloczyn C P
        S = CP @ C.T + R  # innowacja: C P C^T + R
        try:
            S_inv = np.linalg.inv(S)  # próba bezpośredniej odwrotności
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)  # awaryjnie pseudoodwrotność

        P_next = A @ P @ A.T - A @ P @ C.T @ S_inv @ CP @ A.T + Q  # iteracja estymatora
        if np.max(np.abs(P_next - P)) < tol:  # warunek zbieżności
            P = P_next  # aktualizujemy i wychodzimy
            break
        P = P_next  # kolejna iteracja

    S = C @ P @ C.T + R  # finalna macierz innowacji
    try:
        S_inv = np.linalg.inv(S)  # normalna odwrotność
    except np.linalg.LinAlgError:
        S_inv = np.linalg.pinv(S)  # awaryjna pseudoodwrotność

    L = P @ C.T @ S_inv  # wzmocnienie Kalmana: L = P C^T (CPC^T + R)^-1
    return L  # zwracamy stałe wzmocnienie estymatora


class LQGBallbotController(Node):
    def __init__(self):
        super().__init__("lqg_ballbot_controller")  # nazwa noda w ROS2

        self.imu_sub = self.create_subscription(
            Float64MultiArray,  # wiadomość z IMU
            "/imu/kalman_state",  # topic z roll, pitch, p, q, r
            self.imu_callback,  # callback przetwarzający dane z IMU
            10,  # głębokość kolejki ROS
        )

        self.wheel_sub = self.create_subscription(
            Float64MultiArray,  # wiadomość z enkoderów / prędkości kół
            "wheel_state",  # topic z prędkościami i stanem kół
            self.wheel_callback,  # callback przetwarzający pomiar prędkości kół
            10,  # kolejka wiadomości
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,  # komenda sterująca w postaci 3 prędkości kół
            "vel_from_controller",  # topic, który odbiera niższy poziom sterowania silnikami
            10,  # kolejka publikacji
        )

        self.latest_imu: Optional[np.ndarray] = None  # tutaj trzymamy ostatni wektor pomiaru z IMU
        self.latest_wheel_omega = np.zeros(3, dtype=float)  # ostatnie zmierzone prędkości trzech kół

        self.WHEEL_SPEED_MAX = 330.0 * 2.0 * math.pi / 60.0  # maksymalna dopuszczalna prędkość kątowa kół [rad/s]
        self.CONTROL_DT = 0.01  # okres pracy sterownika: 10 ms
        self.SAFE_TILT_LIMIT = math.radians(18.0)  # próg bezpieczeństwa przechyłu; po przekroczeniu zerujemy sterowanie

        # Parametry do strojenia modelu i sterowania
        self.G = 9.80665  # przyspieszenie ziemskie [m/s^2]
        self.COM_HEIGHT = 0.12  # efektywna wysokość środka masy nad punktem kontaktu [m]
        self.WHEEL_ACCEL_GAIN = 1.0  # skalowanie mapowania przyspieszenia na prędkości kół
        self.MOTOR_TAU = 0.05  # stała czasowa uproszczonego modelu silnika
        self.K_YAW = 0.35  # wzmocnienie tłumienia yaw; używane w jądrze macierzy kinematyki

        # Nominalna geometria 3-kółowa, koła rozmieszczone co 120 stopni wokół robota
        beta = np.array([0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0], dtype=float)  # kąty osi kół w układzie robota
        self.Ja = self.WHEEL_ACCEL_GAIN * np.vstack(
            [
                -np.sin(beta),  # pierwszy wiersz macierzy mapuje na składową przyspieszenia x
                np.cos(beta),  # drugi wiersz mapuje na składową przyspieszenia y
            ]
        )  # macierz kinematyczna 2x3: a = J_a * omega
        self.Ja_pinv = self.Ja.T @ np.linalg.inv(self.Ja @ self.Ja.T)  # pseudoodwrotność minimum-norm do przejścia a -> omega
        self.null_vec = np.array([[1.0], [1.0], [1.0]], dtype=float) / 3.0  # wektor z jądra J_a, używany do dodania yaw bez wpływu na balans

        # Stan estymatora Kalmana:
        # x = [roll, p, pitch, q, w1, w2, w3]^T
        # roll  - kąt przechyłu wokół osi x
        # p     - prędkość kątowa wokół osi x z IMU
        # pitch - kąt przechyłu wokół osi y
        # q     - prędkość kątowa wokół osi y z IMU
        # w1..w3 - prędkości trzech kół z enkoderów
        self.x_hat = np.zeros((7, 1), dtype=float)  # estymowany stan początkowo równy zero
        self.P = np.eye(7, dtype=float) * 1.0  # początkowa kowariancja estymatora; mała pewność startowa

        self.u_prev = np.zeros((3, 1), dtype=float)  # poprzednie sterowanie, potrzebne do predykcji stanu

        self.A_c, self.B_c, self.C = self.build_continuous_model()  # budowa modelu ciągłego układu
        self.Ad, self.Bd = euler_discretize(self.A_c, self.B_c, self.CONTROL_DT)  # dyskretyzacja modelu do pracy w czasie próbkowanym

        # LQR tylko dla podukładu przechyłów, czyli części odpowiedzialnej za stabilizację balansu
        self.K_lqr = self.build_lqr_gain()  # macierz sprzężenia zwrotnego stanu

        # Filtr Kalmana dla pełnego stanu:
        # Qk - szum procesu, czyli jak bardzo ufamy modelowi
        # Rk - szum pomiaru, czyli jak bardzo ufamy czujnikom
        self.Qk = np.diag([1e-5, 5e-4, 1e-5, 5e-4, 2e-3, 2e-3, 2e-3])  # szum procesu dla 7 stanów
        self.Rk = np.diag([5e-4, 2e-3, 5e-4, 2e-3, 4e-3, 4e-3, 4e-3])  # szum pomiarowy dla 7 kanałów pomiarowych

        self.Lk = solve_kalman_gain_iterative(self.Ad, self.C, self.Qk, self.Rk)  # wzmocnienie Kalmana; obliczone raz przy starcie

        self.timer = self.create_timer(self.CONTROL_DT, self.control_loop)  # okresowa pętla sterowania wykonywana co CONTROL_DT

    def build_continuous_model(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        A = np.zeros((7, 7), dtype=float)  # macierz A modelu ciągłego; 7 stanów
        B = np.zeros((7, 3), dtype=float)  # macierz B modelu ciągłego; 3 wejścia sterujące (3 koła)
        C = np.eye(7, dtype=float)  # macierz pomiaru; zakładamy, że obserwujemy wszystkie stany bezpośrednio

        g_over_l = self.G / self.COM_HEIGHT  # współczynnik wynikający z liniaryzacji wokół pionu
        inv_l = 1.0 / self.COM_HEIGHT  # 1 / wysokość środka masy; skaluje wpływ sterowania na kąt

        # x = [roll, p, pitch, q, w1, w2, w3]
        # Dla przechyłu wokół osi x:
        #   roll_dot = p
        #   p_dot = (g/l)*roll + wpływ ruchu kół
        A[0, 1] = 1.0  # pochodna roll jest równa prędkości kątowej p
        A[1, 0] = g_over_l  # sprzężenie grawitacyjne: przechył powoduje narastanie p
        A[1, 4:7] = inv_l * self.Ja[0, :]  # wpływ trzech kół na przyspieszenie w osi x

        # Dla przechyłu wokół osi y:
        #   pitch_dot = q
        #   q_dot = (g/l)*pitch + wpływ ruchu kół
        A[2, 3] = 1.0  # pochodna pitch jest równa prędkości q
        A[3, 2] = g_over_l  # sprzężenie grawitacyjne w osi pitch
        A[3, 4:7] = inv_l * self.Ja[1, :]  # wpływ prędkości kół na oś y

        # Uproszczony model silników: każdy napęd zachowuje się jak układ I rzędu
        A[4, 4] = -1.0 / self.MOTOR_TAU  # zanikanie prędkości koła 1 bez sterowania
        A[5, 5] = -1.0 / self.MOTOR_TAU  # zanikanie prędkości koła 2 bez sterowania
        A[6, 6] = -1.0 / self.MOTOR_TAU  # zanikanie prędkości koła 3 bez sterowania

        # Wejścia u są tu traktowane jako referencje sterujące dla prędkości kół
        B[4, 0] = 1.0 / self.MOTOR_TAU  # wejście pierwszego kanału steruje prędkością koła 1
        B[5, 1] = 1.0 / self.MOTOR_TAU  # wejście drugiego kanału steruje prędkością koła 2
        B[6, 2] = 1.0 / self.MOTOR_TAU  # wejście trzeciego kanału steruje prędkością koła 3

        return A, B, C  # zwrot pełnego modelu ciągłego

    def build_lqr_gain(self) -> np.ndarray:
        A_tilt = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],  # roll_dot = p
                [self.G / self.COM_HEIGHT, 0.0, 0.0, 0.0],  # p_dot zależy od roll
                [0.0, 0.0, 0.0, 1.0],  # pitch_dot = q
                [0.0, 0.0, self.G / self.COM_HEIGHT, 0.0],  # q_dot zależy od pitch
            ],
            dtype=float,
        )  # podukład 4x4 odpowiedzialny za balans

        B_tilt = np.array(
            [
                [0.0, 0.0],  # kąty nie reagują bezpośrednio na sterowanie
                [1.0 / self.COM_HEIGHT, 0.0],  # sterowanie w osi x wpływa na p_dot
                [0.0, 0.0],  # pitch nie reaguje bezpośrednio na sterowanie
                [0.0, 1.0 / self.COM_HEIGHT],  # sterowanie w osi y wpływa na q_dot
            ],
            dtype=float,
        )  # macierz wejścia dla stabilizacji przechyłu

        Ad, Bd = euler_discretize(A_tilt, B_tilt, self.CONTROL_DT)  # przejście do wersji dyskretnej, aby liczyć LQR numerycznie

        Q = np.diag([350.0, 25.0, 350.0, 25.0])  # wagi na stany: kąty ważniejsze niż prędkości kątowe
        R = np.diag([1.0, 1.0])  # wagi na sterowanie; większe R oznacza bardziej zachowawcze sterowanie

        P = solve_dare_iterative(Ad, Bd, Q, R)  # rozwiązanie dyskretnego równania Riccatiego dla LQR

        S = R + Bd.T @ P @ Bd  # macierz pośrednia w wzorze na K
        try:
            S_inv = np.linalg.inv(S)  # standardowa odwrotność
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)  # awaryjnie pseudoodwrotność

        K = S_inv @ (Bd.T @ P @ Ad)  # wzór na sprzężenie zwrotne LQR
        return K  # zwracamy macierz K_lqr

    def imu_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:  # sprawdzenie, czy wiadomość ma komplet danych
            return  # zbyt krótka wiadomość jest ignorowana

        roll = float(msg.data[0])  # roll z IMU
        pitch = float(msg.data[1])  # pitch z IMU
        p = float(msg.data[2])  # prędkość kątowa wokół osi x
        q = float(msg.data[3])  # prędkość kątowa wokół osi y
        r = float(msg.data[4])  # prędkość kątowa wokół osi z (yaw)

        self.latest_imu = np.array(
            [wrap_angle(roll), p, wrap_angle(pitch), q, r],  # zawijamy roll i pitch, żeby uniknąć problemów granicznych
            dtype=float,
        )  # zapisujemy najnowszy pomiar z IMU do późniejszego sterowania

    def wheel_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 15:  # oczekujemy dłuższej wiadomości wheel_state z prędkościami w polach 4, 9, 14
            return  # jeśli wiadomość jest niepełna, ignorujemy ją

        self.latest_wheel_omega[0] = float(msg.data[4])  # prędkość kątowa koła 1
        self.latest_wheel_omega[1] = float(msg.data[9])  # prędkość kątowa koła 2
        self.latest_wheel_omega[2] = float(msg.data[14])  # prędkość kątowa koła 3

    def reset_estimator(self) -> None:
        self.x_hat[:] = 0.0  # zerujemy stan estymatora po utracie stabilności
        self.P[:] = np.eye(7, dtype=float) * 1.0  # przywracamy początkową niepewność
        self.u_prev[:] = 0.0  # poprzednie sterowanie też zerujemy

    def publish_zero(self) -> None:
        msg = Float64MultiArray()  # tworzymy pustą wiadomość ROS
        msg.data = [0.0, 0.0, 0.0]  # zero dla trzech kół
        self.cmd_pub.publish(msg)  # publikujemy zerowe sterowanie

    def control_loop(self) -> None:
        if self.latest_imu is None:  # jeśli nie przyszły jeszcze dane z IMU
            self.publish_zero()  # nie sterujemy, bo nie mamy obserwacji
            return  # kończymy iterację

        roll_meas, p_meas, pitch_meas, q_meas, r_meas = self.latest_imu  # rozpakowanie ostatniego pomiaru z IMU
        w_meas = self.latest_wheel_omega.copy()  # lokalna kopia prędkości kół, aby pracować na spójnych danych

        if abs(roll_meas) > self.SAFE_TILT_LIMIT or abs(pitch_meas) > self.SAFE_TILT_LIMIT:
            self.reset_estimator()  # jeśli robot jest zbyt mocno przechylony, resetujemy estymator
            self.publish_zero()  # zabezpieczenie: zatrzymujemy wszystkie koła
            return  # przerywamy sterowanie

        y = np.array(
            [
                [roll_meas],  # pomiar roll
                [p_meas],  # pomiar p
                [pitch_meas],  # pomiar pitch
                [q_meas],  # pomiar q
                [w_meas[0]],  # pomiar prędkości koła 1
                [w_meas[1]],  # pomiar prędkości koła 2
                [w_meas[2]],  # pomiar prędkości koła 3
            ],
            dtype=float,
        )  # wektor obserwacji y = [roll, p, pitch, q, w1, w2, w3]^T

        # Predykcja stanu: przewidujemy, jaki będzie stan za jeden krok czasu na podstawie modelu i poprzedniego sterowania
        x_pred = self.Ad @ self.x_hat + self.Bd @ self.u_prev  # x(k|k-1) = A_d x(k-1|k-1) + B_d u(k-1)
        P_pred = self.Ad @ self.P @ self.Ad.T + self.Qk  # predykcja kowariancji: propagacja niepewności przez model

        # Korekcja estymatora: porównujemy pomiar z przewidywaniem
        S = self.C @ P_pred @ self.C.T + self.Rk  # macierz innowacji: niepewność przewidywania + niepewność pomiaru
        try:
            S_inv = np.linalg.inv(S)  # próbujemy policzyć odwrotność
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)  # awaryjnie używamy pseudoodwrotności

        Kf = P_pred @ self.C.T @ S_inv  # wzmocnienie filtru Kalmana
        innovation = y - self.C @ x_pred  # innowacja, czyli błąd pomiaru względem predykcji
        self.x_hat = x_pred + Kf @ innovation  # korekta estymowanego stanu

        I = np.eye(7, dtype=float)  # macierz jednostkowa potrzebna do stabilnej aktualizacji kowariancji
        self.P = (I - Kf @ self.C) @ P_pred @ (I - Kf @ self.C).T + Kf @ self.Rk @ Kf.T  # postać Josepha, numerycznie stabilniejsza

        # LQR działa tylko na części związanej z przechyłami: roll, p, pitch, q
        x_tilt = self.x_hat[0:4, :]  # wycinamy podwektor stanu odpowiedzialny za balans
        a_cmd = -self.K_lqr @ x_tilt  # sterowanie LQR: u = -Kx

        # Ograniczenie wartości sterowania, żeby nie wygenerować komend nierealnych dla mechaniki
        a_norm = float(np.linalg.norm(a_cmd))  # norma wektora poleceń
        A_CMD_MAX = 4.0  # maksymalna dopuszczalna amplituda przyspieszenia sterującego
        if a_norm > A_CMD_MAX:  # jeśli komenda jest za duża
            a_cmd = a_cmd * (A_CMD_MAX / a_norm)  # skalujemy ją do dozwolonej normy

        # Alokacja sterowania na trzy koła:
        # a_cmd jest przyspieszeniem w płaszczyźnie, a macierz J_a przelicza je na prędkości kół
        omega_planar = self.Ja_pinv @ a_cmd  # rozwiązanie minimalnonormowe dla trzech kół

        # Dodanie składowej yaw:
        # wektor null_vec leży w jądrze J_a, więc można nim regulować obrót wokół osi pionowej,
        # nie zmieniając przyspieszenia odpowiedzialnego za balans
        omega_yaw = -self.K_YAW * r_meas  # prosta stabilizacja yaw-rate przez tłumienie prędkości kątowej z IMU
        omega_yaw_vec = self.null_vec * omega_yaw  # rozkład tej składowej na trzy koła

        omega_ref = omega_planar + omega_yaw_vec  # pełna referencja prędkości dla trzech kół

        # Saturacja globalna: jeśli którakolwiek składowa przekracza bezpieczną prędkość, skalujemy cały wektor
        max_abs = float(np.max(np.abs(omega_ref)))  # największa wartość bezwzględna wśród trzech kół
        if max_abs > self.WHEEL_SPEED_MAX:  # jeśli przekroczyła limit
            omega_ref = omega_ref * (self.WHEEL_SPEED_MAX / max_abs)  # zachowujemy proporcje i ograniczamy maksymalną wartość

        for i in range(3):  # dodatkowa saturacja elementowa, żeby mieć pewność, że każda składowa mieści się w zakresie
            omega_ref[i, 0] = clamp(float(omega_ref[i, 0]), -self.WHEEL_SPEED_MAX, self.WHEEL_SPEED_MAX)

        self.u_prev = omega_ref.copy()  # zapisujemy sterowanie do następnej predykcji estymatora

        out = Float64MultiArray()  # wiadomość wyjściowa ROS
        out.data = [float(omega_ref[0, 0]), float(omega_ref[1, 0]), float(omega_ref[2, 0])]  # trzy zadane prędkości kół
        self.cmd_pub.publish(out)  # publikacja komendy do niższego poziomu sterowania

    def destroy_node(self):
        self.publish_zero()  # przy zamykaniu noda zatrzymujemy układ
        super().destroy_node()  # wywołanie destruktora klasy bazowej ROS2


def main(args=None):
    rclpy.init(args=args)  # inicjalizacja systemu ROS2
    node = LQGBallbotController()  # utworzenie instancji sterownika LQG
    try:
        rclpy.spin(node)  # uruchomienie pętli zdarzeń ROS2 i oczekiwanie na callbacki
    except KeyboardInterrupt:
        pass  # pozwalamy zakończyć program Ctrl+C bez błędu
    finally:
        node.destroy_node()  # sprzątanie zasobów
        rclpy.shutdown()  # zamknięcie ROS2


if __name__ == "__main__":
    main()  # punkt wejścia programu