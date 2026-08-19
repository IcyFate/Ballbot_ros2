#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <tuple>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
constexpr double r_k = 0.0425;

constexpr double K1 = -20.0;
constexpr double K2 = -10.0;
constexpr double K3 = 1.0;
constexpr double K4 = 1.0;

constexpr double K1_STEP = 1.0;
constexpr double K2_STEP = 1.0;
constexpr double K3_STEP = 0.2;
constexpr double K4_STEP = 0.2;

constexpr double POS_SIGN_X = -1.0;
constexpr double POS_SIGN_Y = -1.0;
constexpr double VEL_SIGN_X = -1.0;
constexpr double VEL_SIGN_Y = -1.0;

constexpr double MIN_COMMAND_RAD = 2.2;
constexpr double MAX_W_RAD = 35.0;

constexpr double R_BALL = 0.125;

constexpr double PI_VALUE = 3.14159265358979323846;
constexpr double SQRT3_2 = 0.86602540378;
constexpr double SQRT2_2 = 0.70710678;

constexpr double ANGLE_DEADBAND = 0.0;
constexpr double RATE_DEADBAND = 0.0;
constexpr double POSITION_DEADBAND = 0.0;
constexpr double VELOCITY_DEADBAND = 0.0;

struct TerminalState
{
  int fd{-1};
  FILE * file{nullptr};
  bool close_file{false};
  bool old_settings_valid{false};
  termios old_settings{};
};

TerminalState setup_terminal()
{
  TerminalState terminal;
  terminal.file = ::fopen("/dev/tty", "r");
  if (terminal.file == nullptr) {
    terminal.file = stdin;
  } else {
    terminal.close_file = true;
  }

  terminal.fd = ::fileno(terminal.file);
  if (terminal.fd < 0) {
    throw std::runtime_error("cannot access terminal file descriptor");
  }

  if (::tcgetattr(terminal.fd, &terminal.old_settings) != 0) {
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
  }
  terminal.old_settings_valid = true;

  termios new_settings = terminal.old_settings;
  new_settings.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 0;

  if (::tcsetattr(terminal.fd, TCSADRAIN, &new_settings) != 0) {
    throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
  }

  return terminal;
}

void restore_terminal(TerminalState & terminal)
{
  if (terminal.old_settings_valid) {
    ::tcsetattr(terminal.fd, TCSADRAIN, &terminal.old_settings);
  }
  if (terminal.close_file && terminal.file != nullptr) {
    ::fclose(terminal.file);
  }
}
}  // namespace

class LqrBalanceController : public rclcpp::Node
{
public:
  LqrBalanceController()
  : Node("lqr_balance_controller"),
    last_time_(get_clock()->now())
  {
    imu_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/imu/kalman_state", 1,
      std::bind(&LqrBalanceController::imu_callback, this, std::placeholders::_1));
    wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "wheel_state", 1,
      std::bind(&LqrBalanceController::wheel_callback, this, std::placeholders::_1));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("vel_from_controller", 1);
    msg_.data = {0.0, 0.0, 0.0};

    timer_ = create_wall_timer(
      std::chrono::duration<double>(0.004),
      std::bind(&LqrBalanceController::control_loop, this));

    keyboard_thread_ = std::thread(&LqrBalanceController::keyboard_loop, this);

    RCLCPP_INFO(get_logger(), "LQR tuning node started");
    RCLCPP_INFO(
      get_logger(),
      "Keys: q/a -> K1 +/-1, w/s -> K2 +/-1, e/d -> K3 +/-0.2, r/f -> K4 +/-0.2, x -> exit");
    print_status();
  }

  ~LqrBalanceController() override
  {
    stop_keyboard();
  }

  void stop_keyboard()
  {
    keyboard_stop_.store(true);
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
  }

private:
  void print_status()
  {
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    {
      std::lock_guard<std::mutex> guard(gain_lock_);
      k1 = k1_;
      k2 = k2_;
      k3 = k3_;
      k4 = k4_;
    }

    RCLCPP_INFO(
      get_logger(), "ACTUAL: K1=%.2f, K2=%.2f, K3=%.2f, K4=%.2f",
      k1, k2, k3, k4);
  }

  void keyboard_loop()
  {
    TerminalState terminal;
    bool configured = false;

    try {
      terminal = setup_terminal();
      configured = true;

      while (!keyboard_stop_.load()) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(terminal.fd, &read_set);

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        const int ready = ::select(terminal.fd + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready <= 0) {
          continue;
        }

        char ch = '\0';
        if (::read(terminal.fd, &ch, 1) != 1) {
          continue;
        }

        bool changed = true;
        {
          std::lock_guard<std::mutex> guard(gain_lock_);
          if (ch == 'q') {
            k1_ += K1_STEP;
          } else if (ch == 'a') {
            k1_ -= K1_STEP;
          } else if (ch == 'w') {
            k2_ += K2_STEP;
          } else if (ch == 's') {
            k2_ -= K2_STEP;
          } else if (ch == 'e') {
            k3_ += K3_STEP;
          } else if (ch == 'd') {
            k3_ -= K3_STEP;
          } else if (ch == 'r') {
            k4_ += K4_STEP;
          } else if (ch == 'f') {
            k4_ -= K4_STEP;
          } else if (ch == 'x') {
            RCLCPP_INFO(get_logger(), "Exit requested from keyboard");
            keyboard_stop_.store(true);
            rclcpp::shutdown();
            break;
          } else {
            changed = false;
          }
        }

        if (changed) {
          print_status();
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Keyboard thread error: %s", error.what());
    }

    if (configured) {
      restore_terminal(terminal);
    }
  }

  void imu_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto & d = msg->data;
    if (d.size() < 4) {
      return;
    }

    roll_ = d[0];
    pitch_ = d[1];

    roll_rate_ = d[2];
    pitch_rate_ = d[3];
  }

  void wheel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const auto & d = msg->data;
    if (d.size() < 16) {
      return;
    }

    const double s1_raw = static_cast<double>(d[3]);
    const double s2_raw = static_cast<double>(d[8]);
    const double s3_raw = static_cast<double>(d[13]);

    const double u1_raw = static_cast<double>(d[5]);
    const double u2_raw = static_cast<double>(d[10]);
    const double u3_raw = static_cast<double>(d[15]);

    const double s1 = s3_raw;
    const double s2 = s2_raw;
    const double s3 = s1_raw;

    const double u1 = u3_raw;
    const double u2 = u2_raw;
    const double u3 = u1_raw;

    const double c = SQRT2_2;

    const double psi_pos = -s1 / (R_BALL * c);
    const double phi_pos = (s3 - s2) / (2.0 * SQRT3_2 * R_BALL * c);
    pos_x_meas_ = R_BALL * c * (phi_pos + psi_pos);
    pos_y_meas_ = R_BALL * c * (-phi_pos + psi_pos);

    const double psi_vel = -u1 / (R_BALL * c);
    const double phi_vel = (u3 - u2) / (2.0 * SQRT3_2 * R_BALL * c);
    vel_x_meas_ = R_BALL * c * (phi_vel + psi_vel);
    vel_y_meas_ = R_BALL * c * (-phi_vel + psi_vel);

    if (!pos_x_ref_.has_value()) {
      pos_x_ref_ = pos_x_meas_;
      pos_y_ref_ = pos_y_meas_;
    }
  }

  double deadband(double x, double threshold)
  {
    if (std::abs(x) < threshold) {
      return 0.0;
    }
    return x;
  }

  double min_command_filter(double x)
  {
    if (std::abs(x) < MIN_COMMAND_RAD) {
      return 0.0;
    }
    return x;
  }

  std::tuple<double, double, double> limit_wheels(double w1, double w2, double w3)
  {
    const double max_w = std::max({std::abs(w1), std::abs(w2), std::abs(w3)});

    if (max_w > MAX_W_RAD) {
      const double scale = MAX_W_RAD / max_w;
      w1 *= scale;
      w2 *= scale;
      w3 *= scale;

      cmd_vel_x_ *= scale;
      cmd_vel_y_ *= scale;
    }

    return {w1, w2, w3};
  }

  void control_loop()
  {
    const auto now = get_clock()->now();
    double dt = (now - last_time_).nanoseconds() * 1e-9;
    last_time_ = now;

    if (dt <= 0.0) {
      return;
    }

    if (dt > 0.02) {
      dt = 0.02;
    }

    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    {
      std::lock_guard<std::mutex> guard(gain_lock_);
      k1 = k1_;
      k2 = k2_;
      k3 = k3_;
      k4 = k4_;
    }

    const double theta_x = deadband(pitch_, ANGLE_DEADBAND);
    const double theta_y = deadband(roll_, ANGLE_DEADBAND);

    const double theta_dot_x = deadband(pitch_rate_, RATE_DEADBAND);
    const double theta_dot_y = deadband(roll_rate_, RATE_DEADBAND);

    pos_x_ = pos_x_meas_;
    pos_y_ = pos_y_meas_;

    vel_x_ = vel_x_meas_;
    vel_y_ = vel_y_meas_;

    double px_fb = 0.0;
    double py_fb = 0.0;
    if (pos_x_ref_.has_value() && pos_y_ref_.has_value()) {
      px_fb = deadband(pos_x_meas_ - pos_x_ref_.value(), POSITION_DEADBAND);
      py_fb = deadband(pos_y_meas_ - pos_y_ref_.value(), POSITION_DEADBAND);
    }

    const double vx_fb = deadband(vel_x_meas_, VELOCITY_DEADBAND);
    const double vy_fb = deadband(vel_y_meas_, VELOCITY_DEADBAND);

    const double ax = -(k1 * theta_x + k2 * theta_dot_x +
      k3 * (POS_SIGN_X * px_fb) + k4 * (VEL_SIGN_X * vx_fb));

    const double ay = -(k1 * theta_y + k2 * theta_dot_y +
      k3 * (POS_SIGN_Y * py_fb) + k4 * (VEL_SIGN_Y * vy_fb));

    cmd_vel_x_ += ax * dt;
    cmd_vel_y_ += ay * dt;

    if (
      std::abs(px_fb) < POSITION_DEADBAND && std::abs(py_fb) < POSITION_DEADBAND &&
      std::abs(vx_fb) < VELOCITY_DEADBAND && std::abs(vy_fb) < VELOCITY_DEADBAND &&
      std::abs(theta_x) < ANGLE_DEADBAND && std::abs(theta_y) < ANGLE_DEADBAND &&
      std::abs(theta_dot_x) < RATE_DEADBAND && std::abs(theta_dot_y) < RATE_DEADBAND)
    {
      cmd_vel_x_ = 0.0;
      cmd_vel_y_ = 0.0;
    }

    const double vx_r = 0.70710678 * cmd_vel_x_ - 0.70710678 * cmd_vel_y_;
    const double vy_r = 0.70710678 * cmd_vel_x_ + 0.70710678 * cmd_vel_y_;

    const double V1 = -vy_r * std::cos(PI_VALUE / 4.0);
    const double V2 = (-SQRT3_2 * vx_r + 0.5 * vy_r) * std::cos(PI_VALUE / 4.0);
    const double V3 = (SQRT3_2 * vx_r + 0.5 * vy_r) * std::cos(PI_VALUE / 4.0);

    double w1 = V1 / r_k;
    double w2 = V2 / r_k;
    double w3 = V3 / r_k;

    std::tie(w1, w2, w3) = limit_wheels(w1, w2, w3);

    w1 = min_command_filter(w1);
    w2 = min_command_filter(w2);
    w3 = min_command_filter(w3);

    msg_.data[0] = static_cast<double>(-w3);
    msg_.data[1] = static_cast<double>(-w2);
    msg_.data[2] = static_cast<double>(-w1);

    pub_->publish(msg_);

    log_counter_ += 1;
    if (log_counter_ >= 100) {
      log_counter_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "pitch=%.4f roll=%.4f px_used=%.4f py_used=%.4f vx_meas=%.4f vy_meas=%.4f ax=%.4f ay=%.4f w1=%.2f w2=%.2f w3=%.2f",
        pitch_, roll_, pos_x_, pos_y_, vel_x_meas_, vel_y_meas_, ax, ay, w1, w2, w3);
    }
  }

  double roll_{0.0};
  double pitch_{0.0};
  double roll_rate_{0.0};
  double pitch_rate_{0.0};

  double pos_x_{0.0};
  double pos_y_{0.0};
  double vel_x_{0.0};
  double vel_y_{0.0};

  double pos_x_meas_{0.0};
  double pos_y_meas_{0.0};
  double vel_x_meas_{0.0};
  double vel_y_meas_{0.0};

  std::optional<double> pos_x_ref_;
  std::optional<double> pos_y_ref_;

  double cmd_vel_x_{0.0};
  double cmd_vel_y_{0.0};

  double k1_{K1};
  double k2_{K2};
  double k3_{K3};
  double k4_{K4};
  std::mutex gain_lock_;

  rclcpp::Time last_time_;
  int log_counter_{0};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Float64MultiArray msg_;

  std::atomic<bool> keyboard_stop_{false};
  std::thread keyboard_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LqrBalanceController>();

  try {
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }

  node->stop_keyboard();
  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
