#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <tuple>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{
constexpr double r_k = 0.0425;

constexpr double KP = 5.0;
constexpr double KI = 0.0;
constexpr double KD = 0.6;

constexpr double KP_STEP = 1.0;
constexpr double KI_STEP = 0.2;
constexpr double KD_STEP = 0.2;

constexpr double I_LIMIT = 1.0;

constexpr double MIN_COMMAND_RAD = 2.2;
constexpr double MAX_W_RAD = 35.0;

constexpr double PI_VALUE = 3.14159265358979323846;
constexpr double SQRT3_2 = 0.86602540378;

constexpr double ANGLE_DEADBAND = 0.0;

class AnglePid
{
public:
  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    derivative_ = 0.0;
    initialized_ = false;
  }

  double update(double error, double error_rate, double dt, double kp, double ki, double kd)
  {
    if (!initialized_) {
      prev_error_ = error;
      initialized_ = true;
    }

    integral_ += error * dt;
    integral_ = std::max(-I_LIMIT, std::min(I_LIMIT, integral_));

    derivative_ = error_rate;
    prev_error_ = error;

    return kp * error + ki * integral_ + kd * derivative_;
  }

  void anti_windup(double saturated_output, double kp, double ki, double kd)
  {
    if (ki == 0.0) {
      return;
    }

    integral_ = (saturated_output - kp * prev_error_ - kd * derivative_) / ki;
    integral_ = std::max(-I_LIMIT, std::min(I_LIMIT, integral_));
  }

private:
  double integral_{0.0};
  double prev_error_{0.0};
  double derivative_{0.0};
  bool initialized_{false};
};

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

class PidBalanceController : public rclcpp::Node
{
public:
  PidBalanceController()
  : Node("pid_balance_controller"),
    last_time_(get_clock()->now())
  {
    imu_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/imu/kalman_state", 1,
      std::bind(&PidBalanceController::imu_callback, this, std::placeholders::_1));

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("vel_from_controller", 1);
    msg_.data = {0.0, 0.0, 0.0};

    timer_ = create_wall_timer(
      std::chrono::duration<double>(0.004),
      std::bind(&PidBalanceController::control_loop, this));

    keyboard_thread_ = std::thread(&PidBalanceController::keyboard_loop, this);

    RCLCPP_INFO(get_logger(), "PID balance controller started");
    RCLCPP_INFO(
      get_logger(),
      "Keys: q/a -> Kp +/-1, w/s -> Ki +/-1, e/d -> Kd +/-0.2, x -> exit");
    print_status();
  }

  ~PidBalanceController() override
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
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    {
      std::lock_guard<std::mutex> guard(gain_lock_);
      kp = kp_;
      ki = ki_;
      kd = kd_;
    }

    RCLCPP_INFO(get_logger(), "ACTUAL: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
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
            kp_ += KP_STEP;
          } else if (ch == 'a') {
            kp_ -= KP_STEP;
          } else if (ch == 'w') {
            ki_ += KI_STEP;
          } else if (ch == 's') {
            ki_ -= KI_STEP;
          } else if (ch == 'e') {
            kd_ += KD_STEP;
          } else if (ch == 'd') {
            kd_ -= KD_STEP;
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
    if (d.size() < 2) {
      return;
    }

    roll_ = d[0];
    pitch_ = d[1];

    if (d.size() >= 4) {
      roll_rate_ = d[2];
      pitch_rate_ = d[3];
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

  std::tuple<double, double, double, double> limit_wheels(double w1, double w2, double w3)
  {
    const double max_w = std::max({std::abs(w1), std::abs(w2), std::abs(w3)});
    double scale = 1.0;

    if (max_w > MAX_W_RAD) {
      scale = MAX_W_RAD / max_w;
      w1 *= scale;
      w2 *= scale;
      w3 *= scale;
    }

    return {w1, w2, w3, scale};
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

    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    {
      std::lock_guard<std::mutex> guard(gain_lock_);
      kp = kp_;
      ki = ki_;
      kd = kd_;
    }

    const double theta_x = deadband(pitch_, ANGLE_DEADBAND);
    const double theta_y = deadband(roll_, ANGLE_DEADBAND);
    const double theta_dot_x = pitch_rate_;
    const double theta_dot_y = roll_rate_;

    if (theta_x == 0.0 && theta_y == 0.0) {
      pid_x_.reset();
      pid_y_.reset();

      cmd_vel_x_ = 0.0;
      cmd_vel_y_ = 0.0;
    } else {
      cmd_vel_x_ = pid_x_.update(theta_x, theta_dot_x, dt, kp, ki, kd);
      cmd_vel_y_ = pid_y_.update(theta_y, theta_dot_y, dt, kp, ki, kd);
    }

    double vx_r = 0.70710678 * cmd_vel_x_ - 0.70710678 * cmd_vel_y_;
    double vy_r = 0.70710678 * cmd_vel_x_ + 0.70710678 * cmd_vel_y_;

    double V1 = -vy_r * std::cos(PI_VALUE / 4.0);
    double V2 = (-SQRT3_2 * vx_r + 0.5 * vy_r) * std::cos(PI_VALUE / 4.0);
    double V3 = (SQRT3_2 * vx_r + 0.5 * vy_r) * std::cos(PI_VALUE / 4.0);

    double w1 = V1 / r_k;
    double w2 = V2 / r_k;
    double w3 = V3 / r_k;

    double wheel_scale = 1.0;
    std::tie(w1, w2, w3, wheel_scale) = limit_wheels(w1, w2, w3);

    if (wheel_scale < 1.0) {
      cmd_vel_x_ *= wheel_scale;
      cmd_vel_y_ *= wheel_scale;
      vx_r *= wheel_scale;
      vy_r *= wheel_scale;

      pid_x_.anti_windup(cmd_vel_x_, kp, ki, kd);
      pid_y_.anti_windup(cmd_vel_y_, kp, ki, kd);
    }

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
        "pitch=%.4f roll=%.4f cmd_x=%.4f cmd_y=%.4f vx=%.4f vy=%.4f w1=%.2f w2=%.2f w3=%.2f",
        pitch_, roll_, cmd_vel_x_, cmd_vel_y_, vx_r, vy_r, w1, w2, w3);
    }
  }

  double roll_{0.0};
  double pitch_{0.0};
  double roll_rate_{0.0};
  double pitch_rate_{0.0};
  double cmd_vel_x_{0.0};
  double cmd_vel_y_{0.0};

  double kp_{KP};
  double ki_{KI};
  double kd_{KD};
  std::mutex gain_lock_;

  AnglePid pid_x_;
  AnglePid pid_y_;

  rclcpp::Time last_time_;
  int log_counter_{0};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Float64MultiArray msg_;

  std::atomic<bool> keyboard_stop_{false};
  std::thread keyboard_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PidBalanceController>();

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
