#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace
{
constexpr int ENCODER_PIN_1A = 24;
constexpr int ENCODER_PIN_2A = 25;
constexpr int ENCODER_PIN_3A = 23;

constexpr int PPR = 480;
constexpr double WHEEL_DIAMETER = 0.048;

constexpr unsigned GLITCH_US = 150;
constexpr double PUBLISH_RATE = 1000.0;
constexpr double PI_VALUE = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI_VALUE;

constexpr double PERIOD_EMA_ALPHA_LOW = 0.8;
constexpr double PERIOD_EMA_ALPHA_HIGH = 0.98;

constexpr double OMEGA_RATE_LIMIT = 150.0;

constexpr double MIN_VALID_PERIOD_S = 1e-5;
constexpr double MAX_VALID_PERIOD_S = 1.0;

constexpr double HIGH_SPEED_THRESHOLD = 20.0;

constexpr double DT_ACCUM_MIN = 0.05;
constexpr double STOP_TIMEOUT = 0.1;

double median(std::deque<double> values)
{
  std::sort(values.begin(), values.end());
  const size_t size = values.size();
  if (size == 0) {
    return 0.0;
  }
  if ((size % 2) == 1) {
    return values[size / 2];
  }
  return 0.5 * (values[size / 2 - 1] + values[size / 2]);
}

uint32_t tick_diff(uint32_t start, uint32_t end)
{
  return end - start;
}
}  // namespace

class EncoderOdomNode;

struct EncoderCallbackContext
{
  EncoderOdomNode * node;
  size_t motor_idx;
};

class EncoderOdomNode : public rclcpp::Node
{
public:
  EncoderOdomNode()
  : Node("encoder_odom_node")
  {
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_state", 1);
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("wheel1_speed", 1);
    dir_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "motor_direction", 1,
      std::bind(&EncoderOdomNode::dir_callback, this, std::placeholders::_1));

    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
      throw std::runtime_error("pigpio daemon not running");
    }

    const auto now = std::chrono::steady_clock::now();
    last_edge_time_.fill(now);

    for (int pin : encoder_pins_) {
      set_mode(pi_, pin, PI_INPUT);
      set_pull_up_down(pi_, pin, PI_PUD_UP);
      set_glitch_filter(pi_, pin, GLITCH_US);
    }

    for (size_t i = 0; i < callbacks_.size(); ++i) {
      callback_contexts_[i] = EncoderCallbackContext{this, i};
      callbacks_[i] = callback_ex(
        pi_, encoder_pins_[i], FALLING_EDGE, &EncoderOdomNode::encoder_callback_static,
        &callback_contexts_[i]);
    }

    msg_.data.assign(16, 0.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / PUBLISH_RATE),
      std::bind(&EncoderOdomNode::publish_state, this));
  }

  ~EncoderOdomNode() override
  {
    cleanup();
  }

private:
  static void encoder_callback_static(
    int, unsigned, unsigned, uint32_t tick, void * userdata)
  {
    auto * context = static_cast<EncoderCallbackContext *>(userdata);
    if (context != nullptr && context->node != nullptr) {
      context->node->encoder_callback(context->motor_idx, tick);
    }
  }

  void dir_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3) {
      std::lock_guard<std::mutex> guard(lock_);
      for (size_t i = 0; i < 3; ++i) {
        direction_[i] = static_cast<int>(msg->data[i]);
      }
    }
  }

  void encoder_callback(size_t motor_idx, uint32_t tick)
  {
    std::lock_guard<std::mutex> guard(lock_);
    const int dir_i = direction_[motor_idx];
    if (dir_i == 0) {
      return;
    }

    position_ticks_[motor_idx] += dir_i;
    last_edge_time_[motor_idx] = std::chrono::steady_clock::now();

    if (!last_edge_tick_valid_[motor_idx]) {
      last_edge_tick_us_[motor_idx] = tick;
      last_edge_tick_valid_[motor_idx] = true;
      return;
    }

    const uint32_t dt_us = tick_diff(last_edge_tick_us_[motor_idx], tick);
    last_edge_tick_us_[motor_idx] = tick;

    if (dt_us == 0) {
      return;
    }

    const double period = dt_us * 1e-6;
    if (period < MIN_VALID_PERIOD_S || period > MAX_VALID_PERIOD_S) {
      return;
    }

    period_hist_[motor_idx].push_back(period);
    if (period_hist_[motor_idx].size() > 3) {
      period_hist_[motor_idx].pop_front();
    }

    const double period_med = median(period_hist_[motor_idx]);
    const double omega_est = TWO_PI / (ticks_per_rev_ * period_med);
    const double alpha = omega_est > HIGH_SPEED_THRESHOLD ?
      PERIOD_EMA_ALPHA_HIGH : PERIOD_EMA_ALPHA_LOW;

    if (!period_ema_init_[motor_idx]) {
      period_ema_[motor_idx] = period_med;
      period_ema_init_[motor_idx] = true;
    } else {
      period_ema_[motor_idx] =
        alpha * period_med + (1.0 - alpha) * period_ema_[motor_idx];
    }

    const double omega = TWO_PI / (ticks_per_rev_ * period_ema_[motor_idx]);
    omega_filtered_[motor_idx] = dir_i * omega;
  }

  void publish_state()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - prev_time_).count();
    prev_time_ = now;

  std::array<int64_t, 3> ticks{};
    std::array<double, 3> omega{};
    std::array<std::chrono::steady_clock::time_point, 3> last_edge_time{};
    {
      std::lock_guard<std::mutex> guard(lock_);
      ticks = position_ticks_;
      omega = omega_filtered_;
      last_edge_time = last_edge_time_;
    }

    const auto omega_before_loop = omega;
    for (size_t i = 0; i < 3; ++i) {
      if (std::chrono::duration<double>(now - last_edge_time[i]).count() > STOP_TIMEOUT) {
        omega[i] = 0.0;
        continue;
      }

      const int64_t delta_ticks = ticks[i] - prev_ticks_[i];
      dt_accum_[i] += dt;
      tick_accum_[i] += delta_ticks;

      if (dt_accum_[i] >= DT_ACCUM_MIN) {
        const double omega_dt =
          (tick_accum_[i] / ticks_per_rev_) * TWO_PI / dt_accum_[i];

        dt_accum_[i] = 0.0;
        tick_accum_[i] = 0;

        if (std::abs(omega[i]) > HIGH_SPEED_THRESHOLD) {
          omega[i] = 0.5 * omega[i] + 0.5 * omega_dt;
        }
      }

      const double max_step = OMEGA_RATE_LIMIT * dt;
      const double diff = omega[i] - omega_before_loop[i];

      if (std::abs(diff) > max_step) {
        omega[i] = omega_before_loop[i] + std::copysign(max_step, diff);
      }

      omega[i] = 0.8 * omega[i] + 0.2 * omega_before_loop[i];
    }

    prev_ticks_ = ticks;
    {
      std::lock_guard<std::mutex> guard(lock_);
      omega_filtered_ = omega;
    }

    const double now_s = get_clock()->now().nanoseconds() * 1e-9;
    msg_.data[0] = static_cast<double>(now_s);

    for (size_t i = 0; i < 3; ++i) {
      const double rev = ticks[i] / ticks_per_rev_;
      const double angle = rev * TWO_PI;
      const double distance = rev * wheel_circ_;
      const double linear_vel = omega[i] * wheel_circ_ / TWO_PI;

      const size_t base = 1 + i * 5;
      msg_.data[base + 0] = static_cast<double>(ticks[i]);
      msg_.data[base + 1] = angle;
      msg_.data[base + 2] = distance;
      msg_.data[base + 3] = omega[i];
      msg_.data[base + 4] = linear_vel;
    }

    speed_msg_.data = static_cast<double>(omega[0]);
    speed_pub_->publish(speed_msg_);
    publisher_->publish(msg_);
  }

  void cleanup()
  {
    if (cleaned_up_) {
      return;
    }
    cleaned_up_ = true;

    for (int callback_id : callbacks_) {
      if (callback_id >= 0) {
        callback_cancel(callback_id);
      }
    }

    if (pi_ >= 0) {
      pigpio_stop(pi_);
      pi_ = -1;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr dir_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int pi_{-1};
  bool cleaned_up_{false};

  std::array<unsigned, 3> encoder_pins_{
    ENCODER_PIN_1A, ENCODER_PIN_2A, ENCODER_PIN_3A};
  std::array<int, 3> callbacks_{{-1, -1, -1}};
  std::array<EncoderCallbackContext, 3> callback_contexts_{};

  std::mutex lock_;
  std::array<int, 3> direction_{{0, 0, 0}};
  std::array<int64_t, 3> position_ticks_{{0, 0, 0}};
  std::array<uint32_t, 3> last_edge_tick_us_{{0, 0, 0}};
  std::array<bool, 3> last_edge_tick_valid_{{false, false, false}};
  std::array<std::chrono::steady_clock::time_point, 3> last_edge_time_{};
  std::array<std::deque<double>, 3> period_hist_{};
  std::array<double, 3> period_ema_{{0.0, 0.0, 0.0}};
  std::array<bool, 3> period_ema_init_{{false, false, false}};
  std::array<int64_t, 3> prev_ticks_{{0, 0, 0}};
  std::chrono::steady_clock::time_point prev_time_{std::chrono::steady_clock::now()};
  std::array<double, 3> dt_accum_{{0.0, 0.0, 0.0}};
  std::array<int64_t, 3> tick_accum_{{0, 0, 0}};
  std::array<double, 3> omega_filtered_{{0.0, 0.0, 0.0}};

  double ticks_per_rev_{static_cast<double>(PPR)};
  double wheel_circ_{PI_VALUE * WHEEL_DIAMETER};
  std_msgs::msg::Float64MultiArray msg_;
  std_msgs::msg::Float64 speed_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<EncoderOdomNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("encoder_read"), "%s", error.what());
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
