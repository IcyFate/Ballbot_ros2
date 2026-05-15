#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# MODEL

r_k = 0.024

# LQR

K1 = -50.0
K2 = -25
K3 = 1.5
K4 = 3.5

# ZNAKI SPRZĘŻENIA TRANSLACYJNEGO
# Jeśli robot ucieka zamiast wracać, najpierw odwróć jedną z tych stałych.
# Nie zmieniaj wszystkiego naraz.
POS_SIGN_X = -1.0
POS_SIGN_Y = -1.0
VEL_SIGN_X = -1.0
VEL_SIGN_Y = -1.0

# SILNIKI

MIN_COMMAND_RAD = 0

# GEOMETRIA

R_BALL = 0.125

SQRT3_2 = 0.86602540378
SQRT2_2 = 0.70710678

MAX_ACC = 7.0
MAX_VEL = 0.8

ANGLE_DEADBAND = 0.005
RATE_DEADBAND = 0.01
POSITION_DEADBAND = 0.005
VELOCITY_DEADBAND = 0.01

# NODE

class LqrBalanceController(Node):

    def __init__(self):
        super().__init__('lqr_balance_controller')

        self.roll = 0.0
        self.pitch = 0.0

        self.roll_rate = 0.0
        self.pitch_rate = 0.0

        self.pos_x = 0.0
        self.pos_y = 0.0

        self.vel_x = 0.0
        self.vel_y = 0.0

        self.pos_x_meas = 0.0
        self.pos_y_meas = 0.0

        self.vel_x_meas = 0.0
        self.vel_y_meas = 0.0

        # punkt odniesienia dla pozycji z enkoderów
        self.pos_x_ref = None
        self.pos_y_ref = None

        # zadana prędkość robota po całkowaniu przyspieszenia z regulatora
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0

        self.last_time = self.get_clock().now()
        self.log_counter = 0

        self.create_subscription(
            Float64MultiArray,
            '/imu/kalman_state',
            self.imu_callback,
            1
        )

        self.create_subscription(
            Float64MultiArray,
            'wheel_state',
            self.wheel_callback,
            1
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            'vel_from_controller',
            1
        )

        self.msg = Float64MultiArray()
        self.msg.data = [0.0, 0.0, 0.0]

        self.timer = self.create_timer(
            0.004,
            self.control_loop
        )

    def imu_callback(self, msg):

        d = msg.data

        if len(d) < 4:
            return

        self.roll = d[0]
        self.pitch = d[1]

        self.roll_rate = d[2]
        self.pitch_rate = d[3]

    def wheel_callback(self, msg):

        d = msg.data

        if len(d) < 16:
            return

        # pozycje kół z enkoderów [m] - surowe kanały
        s1_raw = float(d[3])
        s2_raw = float(d[8])
        s3_raw = float(d[13])

        # prędkości liniowe kół z enkoderów [m/s] - surowe kanały
        u1_raw = float(d[5])
        u2_raw = float(d[10])
        u3_raw = float(d[15])

        # zamiana w3 <-> w1 tak samo jak przy zadawaniu prędkości
        s1 = s3_raw
        s2 = s2_raw
        s3 = s1_raw

        u1 = u3_raw
        u2 = u2_raw
        u3 = u1_raw

        c = SQRT2_2

        # pozycja platformy w układzie robota [m]
        psi_pos = -s1 / (R_BALL * c)
        phi_pos = (s3 - s2) / (2.0 * SQRT3_2 * R_BALL * c)
        self.pos_x_meas = R_BALL * c * (phi_pos + psi_pos)
        self.pos_y_meas = R_BALL * c * (-phi_pos + psi_pos)

        # prędkość platformy w układzie robota [m/s]
        psi_vel = -u1 / (R_BALL * c)
        phi_vel = (u3 - u2) / (2.0 * SQRT3_2 * R_BALL * c)
        self.vel_x_meas = R_BALL * c * (phi_vel + psi_vel)
        self.vel_y_meas = R_BALL * c * (-phi_vel + psi_vel)

        # inicjalizacja punktu odniesienia po pierwszym poprawnym pomiarze
        if self.pos_x_ref is None:
            self.pos_x_ref = self.pos_x_meas
            self.pos_y_ref = self.pos_y_meas

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def deadband(self, x, threshold):

        if abs(x) < threshold:
            return 0.0

        return x

    def min_command_filter(self, x):

        if abs(x) < MIN_COMMAND_RAD:
            return 0.0

        return x

    def control_loop(self):

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        if dt > 0.02:
            dt = 0.02

        # FILTR MAŁYCH DRGAŃ

        theta_x = self.deadband(self.pitch, ANGLE_DEADBAND)
        theta_y = self.deadband(self.roll, ANGLE_DEADBAND)

        theta_dot_x = self.deadband(self.pitch_rate, RATE_DEADBAND)
        theta_dot_y = self.deadband(self.roll_rate, RATE_DEADBAND)

        # STAN Z ENKODERÓW

        self.pos_x = self.pos_x_meas
        self.pos_y = self.pos_y_meas

        self.vel_x = self.vel_x_meas
        self.vel_y = self.vel_y_meas

        if self.pos_x_ref is None or self.pos_y_ref is None:
            px_fb = 0.0
            py_fb = 0.0
        else:
            px_fb = self.deadband(self.pos_x_meas - self.pos_x_ref, POSITION_DEADBAND)
            py_fb = self.deadband(self.pos_y_meas - self.pos_y_ref, POSITION_DEADBAND)

        vx_fb = self.deadband(self.vel_x_meas, VELOCITY_DEADBAND)
        vy_fb = self.deadband(self.vel_y_meas, VELOCITY_DEADBAND)

        # Sprzężenie translacyjne z jawnie wystawionym znakiem.
        # Jeśli robot odjeżdża, najpierw odwróć POS_SIGN_X / POS_SIGN_Y albo VEL_SIGN_X / VEL_SIGN_Y.
        ax = -(K1 * theta_x + K2 * theta_dot_x
               + K3 * (POS_SIGN_X * px_fb)
               + K4 * (VEL_SIGN_X * vx_fb))

        ay = -(K1 * theta_y + K2 * theta_dot_y
               + K3 * (POS_SIGN_Y * py_fb)
               + K4 * (VEL_SIGN_Y * vy_fb))

        # LIMIT ACC NA NORMĘ WEKTORA

        acc_norm = math.sqrt(ax * ax + ay * ay)

        if acc_norm > MAX_ACC:
            scale = MAX_ACC / acc_norm
            ax *= scale
            ay *= scale

        # INTEGRACJA PRZYSPIESZENIA DO ZADANEJ PRĘDKOŚCI

        self.cmd_vel_x += ax * dt
        self.cmd_vel_y += ay * dt

        self.cmd_vel_x = self.clamp(self.cmd_vel_x, -MAX_VEL, MAX_VEL)
        self.cmd_vel_y = self.clamp(self.cmd_vel_y, -MAX_VEL, MAX_VEL)

        # martwa strefa przy bardzo małych odchyłkach
        if abs(px_fb) < POSITION_DEADBAND and abs(py_fb) < POSITION_DEADBAND and \
           abs(vx_fb) < VELOCITY_DEADBAND and abs(vy_fb) < VELOCITY_DEADBAND and \
           abs(theta_x) < ANGLE_DEADBAND and abs(theta_y) < ANGLE_DEADBAND and \
           abs(theta_dot_x) < RATE_DEADBAND and abs(theta_dot_y) < RATE_DEADBAND:
            self.cmd_vel_x = 0.0
            self.cmd_vel_y = 0.0

        # MAPOWANIE KOŁA

        vx_r = 0.70710678 * self.cmd_vel_x - 0.70710678 * self.cmd_vel_y
        vy_r = 0.70710678 * self.cmd_vel_x + 0.70710678 * self.cmd_vel_y

        V1 = -vy_r * math.cos(math.pi / 4)
        V2 = (-SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)
        V3 = (SQRT3_2 * vx_r + 0.5 * vy_r) * math.cos(math.pi / 4)

        # m/s -> rad/s

        w1 = V1 / r_k
        w2 = V2 / r_k
        w3 = V3 / r_k

        # MIN PWM FILTER

        w1 = self.min_command_filter(w1)
        w2 = self.min_command_filter(w2)
        w3 = self.min_command_filter(w3)

        # PUB

        self.msg.data[0] = float(-w3)
        self.msg.data[1] = float(-w2)
        self.msg.data[2] = float(-w1)

        self.pub.publish(self.msg)

        # LOGI

        self.log_counter += 1

        if self.log_counter >= 25:
            self.log_counter = 0

            self.get_logger().info(
                f'pitch={self.pitch:.4f} '
                f'roll={self.roll:.4f} '
                f'px_enc={self.pos_x_meas:.4f} '
                f'py_enc={self.pos_y_meas:.4f} '
                f'px_ref={(self.pos_x_ref if self.pos_x_ref is not None else 0.0):.4f} '
                f'py_ref={(self.pos_y_ref if self.pos_y_ref is not None else 0.0):.4f} '
                f'px_used={self.pos_x:.4f} '
                f'py_used={self.pos_y:.4f} '
                f'vx_meas={self.vel_x_meas:.4f} '
                f'vy_meas={self.vel_y_meas:.4f} '
                f'vx_cmd={self.cmd_vel_x:.4f} '
                f'vy_cmd={self.cmd_vel_y:.4f} '
                f'vx_r={vx_r:.4f} '
                f'vy_r={vy_r:.4f} '
                f'ax={ax:.4f} '
                f'ay={ay:.4f} '
                f'acc_norm={acc_norm:.4f} '
                f'w1={w1:.2f} '
                f'w2={w2:.2f} '
                f'w3={w3:.2f}'
            )

# MAIN

def main(args=None):

    rclpy.init(args=args)

    node = LqrBalanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()