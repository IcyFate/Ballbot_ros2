#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace
{
constexpr size_t WHEEL_COUNT = 3;
constexpr std::array<size_t, WHEEL_COUNT> WHEEL_STATE_ANGULAR_VEL_IDXS{{4, 9, 14}};

constexpr std::array<unsigned, WHEEL_COUNT> PIN_RPWM{{10, 4, 6}};
constexpr std::array<unsigned, WHEEL_COUNT> PIN_LPWM{{9, 17, 13}};
constexpr std::array<unsigned, WHEEL_COUNT> PIN_REN{{11, 8, 19}};
constexpr std::array<unsigned, WHEEL_COUNT> PIN_LEN{{5, 22, 26}};

constexpr unsigned PWM_FREQ = 20000;
constexpr unsigned PWM_RANGE = 255;

constexpr double PWM_START_MOVE = 22.0;
constexpr double PWM_MAX = 255.0;

constexpr double REF_DEADBAND_OMEGA = 0.05;
constexpr double DT_MIN = 1e-3;
constexpr double DT_MAX = 0.05;

double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

class PIController
{
public:
  PIController(double kp, double ki, double i_limit = 120.0)
  : kp_(kp),
    ki_(ki),
    i_limit_(i_limit)
  {
  }

  std::pair<double, double> update(double setpoint, double measurement, double dt)
  {
    dt = clamp(dt, DT_MIN, DT_MAX);

    const double error = setpoint - measurement;
    if (!initialized_) {
      initialized_ = true;
    }

    integral_ += error * dt;
    integral_ = clamp(integral_, -i_limit_, i_limit_);

    const double p = kp_ * error;
    const double temp = p + ki_ * integral_;

    double u = temp;
    if (temp > PWM_MAX) {
      u = PWM_MAX;
      if (ki_ != 0.0) {
        integral_ = clamp((PWM_MAX - p) / ki_, -i_limit_, i_limit_);
      }
    } else if (temp < 0.0) {
      u = 0.0;
      if (ki_ != 0.0) {
        integral_ = clamp((0.0 - p) / ki_, -i_limit_, i_limit_);
      }
    }

    return {u, error};
  }

  void reset()
  {
    integral_ = 0.0;
    initialized_ = false;
  }

private:
  double kp_;
  double ki_;
  double i_limit_;
  double integral_{0.0};
  bool initialized_{false};
};
}  // namespace

class WheelVelocityMotorNode : public rclcpp::Node
{
public:
  WheelVelocityMotorNode()
  : Node("wheel_velocity_motor_node"),
    last_time_(get_clock()->now())
  {
    ref_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "vel_from_controller", 1,
      std::bind(&WheelVelocityMotorNode::ref_callback, this, std::placeholders::_1));
    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "wheel_state", 1,
      std::bind(&WheelVelocityMotorNode::state_callback, this, std::placeholders::_1));
    dir_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("motor_direction", 1);

    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
      throw std::runtime_error("pigpiod not running");
    }

    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
      gpio_write(pi_, PIN_REN[i], 1);
      gpio_write(pi_, PIN_LEN[i], 1);
      set_PWM_frequency(pi_, PIN_RPWM[i], PWM_FREQ);
      set_PWM_frequency(pi_, PIN_LPWM[i], PWM_FREQ);
      set_PWM_range(pi_, PIN_RPWM[i], PWM_RANGE);
      set_PWM_range(pi_, PIN_LPWM[i], PWM_RANGE);
    }

    dir_msg_.data = {0, 0, 0};
    stop_all();
  }

  ~WheelVelocityMotorNode() override
  {
    cleanup();
  }

private:
  void ref_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < WHEEL_COUNT) {
      return;
    }

    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
      ref_vel_[i] = static_cast<double>(msg->data[i]);
    }
  }

  int apply_motor(size_t i, double duty, int direction)
  {
    const int duty_int = static_cast<int>(clamp(duty, 0.0, PWM_MAX));

    if (direction > 0) {
      set_PWM_dutycycle(pi_, PIN_LPWM[i], 0);
      set_PWM_dutycycle(pi_, PIN_RPWM[i], duty_int);
    } else if (direction < 0) {
      set_PWM_dutycycle(pi_, PIN_RPWM[i], 0);
      set_PWM_dutycycle(pi_, PIN_LPWM[i], duty_int);
    } else {
      set_PWM_dutycycle(pi_, PIN_RPWM[i], 0);
      set_PWM_dutycycle(pi_, PIN_LPWM[i], 0);
    }

    return duty_int;
  }

  void stop_all()
  {
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
      apply_motor(i, 0.0, 0);
      pi_ctrl_[i].reset();
      last_direction_[i] = 0;
    }

    dir_msg_.data = {0, 0, 0};
    dir_pub_->publish(dir_msg_);
  }

  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 16) {
      return;
    }

    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
      meas_vel_[i] = static_cast<double>(msg->data[WHEEL_STATE_ANGULAR_VEL_IDXS[i]]);
    }

    const auto now = get_clock()->now();
    double dt = (now - last_time_).nanoseconds() * 1e-9;
    last_time_ = now;
    dt = clamp(dt, DT_MIN, DT_MAX);

    std::array<int32_t, WHEEL_COUNT> dir_out{{0, 0, 0}};

    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
      const double ref = ref_vel_[i];
      const double meas = meas_vel_[i];

      if (std::abs(ref) < REF_DEADBAND_OMEGA) {
        pi_ctrl_[i].reset();
        apply_motor(i, 0.0, 0);
        dir_out[i] = 0;
        continue;
      }

      const int direction = ref > 0.0 ? 1 : -1;

      if (direction != last_direction_[i] && last_direction_[i] != 0) {
        pi_ctrl_[i].reset();
      }

      last_direction_[i] = direction;

      const double ref_abs = std::abs(ref);
      const double meas_abs = std::abs(meas);

      const auto [u, error] = pi_ctrl_[i].update(ref_abs, meas_abs, dt);
      (void)error;

      double duty = clamp(u, 0.0, PWM_MAX);
      if (duty > 0.0) {
        duty = std::max(PWM_START_MOVE, duty);
      }

      dir_out[i] = direction;
      apply_motor(i, duty, direction);
    }

    dir_msg_.data.assign(dir_out.begin(), dir_out.end());
    dir_pub_->publish(dir_msg_);
  }

  void cleanup()
  {
    if (cleaned_up_) {
      return;
    }
    cleaned_up_ = true;

    if (pi_ >= 0) {
      stop_all();
      pigpio_stop(pi_);
      pi_ = -1;
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr dir_pub_;

  std::array<double, WHEEL_COUNT> ref_vel_{{0.0, 0.0, 0.0}};
  std::array<double, WHEEL_COUNT> meas_vel_{{0.0, 0.0, 0.0}};
  rclcpp::Time last_time_;
  std::array<int, WHEEL_COUNT> last_direction_{{0, 0, 0}};
  std::array<PIController, WHEEL_COUNT> pi_ctrl_{{
    PIController(3.5, 23.0, 120.0),
    PIController(3.5, 23.0, 120.0),
    PIController(3.5, 23.0, 120.0)}};

  int pi_{-1};
  bool cleaned_up_{false};
  std_msgs::msg::Int32MultiArray dir_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<WheelVelocityMotorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("velocity_controller"), "%s", error.what());
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
