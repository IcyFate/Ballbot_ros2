#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <utility>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace imu_filter
{
constexpr uint8_t I2C_ADDRESS = 0x6A;
constexpr uint8_t WHO_AM_I_REG = 0x0F;
constexpr uint8_t WHO_AM_I_LSM6DSO32 = 0x6C;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11;
constexpr uint8_t CTRL3_C = 0x12;
constexpr uint8_t CTRL9_XL = 0x18;
constexpr uint8_t BURST_START_REG = 0x22;
constexpr size_t BURST_LEN = 12;

constexpr double PI_VALUE = 3.14159265358979323846;
constexpr double GYRO_LSB_TO_RAD_S = 0.00875 * PI_VALUE / 180.0;
constexpr double ACC_LSB_TO_MS2 = 0.244e-3 * 9.80665;

constexpr double ROLL_OFFSET = 0.006;
constexpr double PITCH_OFFSET = -0.025;

constexpr double PI = PI_VALUE;
constexpr double TWO_PI = 2.0 * PI_VALUE;

constexpr double G = 9.80665;
constexpr double ACC_K = 0.005;
constexpr double ACC_FULL_TRUST_ERROR = 0.25;
constexpr double ACC_NO_TRUST_ERROR = 1.0;

class I2cDevice
{
public:
  I2cDevice(const std::string & device, uint8_t address)
  : address_(address)
  {
    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open " + device + ": " + std::strerror(errno));
    }

    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
      const std::string error = std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("cannot select I2C address: " + error);
    }
  }

  ~I2cDevice()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  I2cDevice(const I2cDevice &) = delete;
  I2cDevice & operator=(const I2cDevice &) = delete;

  uint8_t read_reg(uint8_t reg)
  {
    uint8_t value = 0;
    read_burst(reg, &value, 1);
    return value;
  }

  void write_reg(uint8_t reg, uint8_t value)
  {
    uint8_t buffer[2] = {reg, value};
    if (::write(fd_, buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer))) {
      throw std::runtime_error("I2C write failed: " + std::string(std::strerror(errno)));
    }
  }

  bool read_burst(uint8_t reg, uint8_t * buffer, size_t length)
  {
    i2c_msg messages[2] = {};
    messages[0].addr = address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<decltype(messages[1].len)>(length);
    messages[1].buf = buffer;

    i2c_rdwr_ioctl_data packets = {};
    packets.msgs = messages;
    packets.nmsgs = 2;

    return ::ioctl(fd_, I2C_RDWR, &packets) >= 0;
  }

private:
  int fd_{-1};
  uint8_t address_;
};

int16_t le_i16(const uint8_t * buffer, size_t offset)
{
  return static_cast<int16_t>(static_cast<uint16_t>(buffer[offset]) |
    (static_cast<uint16_t>(buffer[offset + 1]) << 8));
}

void initialize_lsm6dso32(I2cDevice & i2c)
{
  const uint8_t who_am_i = i2c.read_reg(WHO_AM_I_REG);
  if (who_am_i != WHO_AM_I_LSM6DSO32) {
    throw std::runtime_error("failed to find LSM6DSO32, WHO_AM_I=" + std::to_string(who_am_i));
  }

  i2c.write_reg(CTRL3_C, 0x01);
  for (int i = 0; i < 100; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if ((i2c.read_reg(CTRL3_C) & 0x01) == 0) {
      break;
    }
  }

  i2c.write_reg(CTRL3_C, 0x44);
  i2c.write_reg(CTRL9_XL, i2c.read_reg(CTRL9_XL) | 0x02);

  i2c.write_reg(CTRL1_XL, 0x58);
  i2c.write_reg(CTRL2_G, 0x50);
}

class TiltEkf
{
public:
  std::array<double, 8> step(double ax, double ay, double az, double gx, double gy, double gz, double dt)
  {
    if (!initialized_) {
      roll_ = std::atan2(ay, az);
      pitch_ = std::atan2(-ax, std::sqrt(ay * ay + az * az));

      bx_ = 0.0;
      by_ = 0.0;
      bz_ = 0.0;

      initialized_ = true;
    }

    const double bx = bx_;
    const double by = by_;
    const double bz = bz_;

    const double p = gx - bx;
    const double q = gy - by;
    const double r = gz - bz;

    double roll = roll_;
    double pitch = pitch_;

    const double sphi = std::sin(roll);
    const double cphi = std::cos(roll);
    const double tth = std::tan(pitch);

    roll += dt * (p + sphi * tth * q + cphi * tth * r);
    pitch += dt * (cphi * q - sphi * r);

    const double acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
    const double acc_error = std::abs(acc_norm - G);

    if (acc_error < ACC_NO_TRUST_ERROR) {
      const double roll_acc = std::atan2(ay, az);
      const double pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

      double y0 = roll_acc - roll;
      if (y0 > PI) {
        y0 -= TWO_PI;
      } else if (y0 < -PI) {
        y0 += TWO_PI;
      }

      const double y1 = pitch_acc - pitch;

      double k_acc = ACC_K;
      if (acc_error > ACC_FULL_TRUST_ERROR) {
        const double scale = (ACC_NO_TRUST_ERROR - acc_error) /
          (ACC_NO_TRUST_ERROR - ACC_FULL_TRUST_ERROR);
        k_acc = ACC_K * scale;
      }

      roll += k_acc * y0;
      pitch += k_acc * y1;
    }

    roll_ = roll;
    pitch_ = pitch;

    return {roll, pitch, p, q, r, bx, by, bz};
  }

private:
  double roll_{0.0};
  double pitch_{0.0};
  double bx_{0.0};
  double by_{0.0};
  double bz_{0.0};
  bool initialized_{false};
};
}  // namespace imu_filter

namespace encoder_odom
{
constexpr int ENCODER_PIN_1A = 24;
constexpr int ENCODER_PIN_2A = 25;
constexpr int ENCODER_PIN_3A = 23;

constexpr int PPR = 480;
constexpr double WHEEL_DIAMETER = 0.085;

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
}  // namespace encoder_odom

namespace wheel_velocity
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
}  // namespace wheel_velocity

namespace balance_pid
{
constexpr double r_k = 0.0425;

constexpr double KP = 5.0;
constexpr double KI = 0.2;
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
constexpr double CONTROL_PERIOD_S = 0.004;

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
}  // namespace balance_pid

class BallbotCombinedNode : public rclcpp::Node
{
public:
  BallbotCombinedNode()
  : Node("ballbot_combined_node"),
    imu_i2c_("/dev/i2c-1", imu_filter::I2C_ADDRESS),
    imu_last_time_(std::chrono::steady_clock::now()),
    wheel_last_time_(get_clock()->now()),
    pid_last_time_(get_clock()->now())
  {
    publish_compat_topics_ = declare_parameter<bool>("publish_compat_topics", true);

    if (publish_compat_topics_) {
      imu_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/imu/kalman_state", 1);
      wheel_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_state", 1);
      wheel1_speed_pub_ = create_publisher<std_msgs::msg::Float64>("wheel1_speed", 1);
      vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("vel_from_controller", 1);
      dir_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("motor_direction", 1);
    }

    imu_filter::initialize_lsm6dso32(imu_i2c_);

    pi_ = pigpio_start(nullptr, nullptr);
    if (pi_ < 0) {
      throw std::runtime_error("pigpiod not running");
    }

    setup_encoder_gpio();
    setup_motor_gpio();

    imu_msg_.data.assign(8, 0.0);
    encoder_msg_.data.assign(16, 0.0);
    pid_msg_.data = {0.0, 0.0, 0.0};
    dir_msg_.data = {0, 0, 0};

    encoder_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pid_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    encoder_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / encoder_odom::PUBLISH_RATE),
      std::bind(&BallbotCombinedNode::encoder_publish_state, this),
      encoder_group_);

    pid_timer_ = create_wall_timer(
      std::chrono::duration<double>(balance_pid::CONTROL_PERIOD_S),
      std::bind(&BallbotCombinedNode::pid_control_loop, this),
      pid_group_);

    wheel_stop_all();

    imu_thread_ = std::thread(&BallbotCombinedNode::imu_loop, this);
    keyboard_thread_ = std::thread(&BallbotCombinedNode::keyboard_loop, this);

    RCLCPP_INFO(get_logger(), "Combined ballbot node started");
    RCLCPP_INFO(
      get_logger(),
      "publish_compat_topics=%s",
      publish_compat_topics_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "Keys: q/a -> Kp +/-1, w/s -> Ki +/-1, e/d -> Kd +/-0.2, x -> exit");
    pid_print_status();
  }

  ~BallbotCombinedNode() override
  {
    cleanup();
  }

  void cleanup()
  {
    if (cleaned_up_.exchange(true)) {
      return;
    }

    imu_stop_.store(true);
    keyboard_stop_.store(true);

    if (encoder_timer_) {
      encoder_timer_->cancel();
    }
    if (pid_timer_) {
      pid_timer_->cancel();
    }

    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }

    if (pi_ >= 0) {
      wheel_stop_all();

      for (int callback_id : encoder_callbacks_) {
        if (callback_id >= 0) {
          callback_cancel(callback_id);
        }
      }

      pigpio_stop(pi_);
      pi_ = -1;
    }
  }

private:
  struct EncoderCallbackContext
  {
    BallbotCombinedNode * node{nullptr};
    size_t motor_idx{0};
  };

  static void encoder_callback_static(
    int, unsigned, unsigned, uint32_t tick, void * userdata)
  {
    auto * context = static_cast<EncoderCallbackContext *>(userdata);
    if (context != nullptr && context->node != nullptr) {
      context->node->encoder_callback(context->motor_idx, tick);
    }
  }

  void setup_encoder_gpio()
  {
    const auto now = std::chrono::steady_clock::now();
    encoder_last_edge_time_.fill(now);

    for (int pin : encoder_pins_) {
      set_mode(pi_, pin, PI_INPUT);
      set_pull_up_down(pi_, pin, PI_PUD_UP);
      set_glitch_filter(pi_, pin, encoder_odom::GLITCH_US);
    }

    for (size_t i = 0; i < encoder_callbacks_.size(); ++i) {
      encoder_callback_contexts_[i] = EncoderCallbackContext{this, i};
      encoder_callbacks_[i] = callback_ex(
        pi_, encoder_pins_[i], FALLING_EDGE, &BallbotCombinedNode::encoder_callback_static,
        &encoder_callback_contexts_[i]);
    }
  }

  void setup_motor_gpio()
  {
    for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
      gpio_write(pi_, wheel_velocity::PIN_REN[i], 1);
      gpio_write(pi_, wheel_velocity::PIN_LEN[i], 1);
      set_PWM_frequency(pi_, wheel_velocity::PIN_RPWM[i], wheel_velocity::PWM_FREQ);
      set_PWM_frequency(pi_, wheel_velocity::PIN_LPWM[i], wheel_velocity::PWM_FREQ);
      set_PWM_range(pi_, wheel_velocity::PIN_RPWM[i], wheel_velocity::PWM_RANGE);
      set_PWM_range(pi_, wheel_velocity::PIN_LPWM[i], wheel_velocity::PWM_RANGE);
    }
  }

  void imu_loop()
  {
    while (!imu_stop_.load() && rclcpp::ok()) {
      imu_loop_once();
    }
  }

  void imu_loop_once()
  {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - imu_last_time_).count();
    imu_last_time_ = now;

    if (dt <= 0.0) {
      dt = 0.001;
    }

    uint8_t rx[imu_filter::BURST_LEN] = {};
    if (!imu_i2c_.read_burst(imu_filter::BURST_START_REG, rx, imu_filter::BURST_LEN)) {
      return;
    }

    const int16_t gx_raw = imu_filter::le_i16(rx, 0);
    const int16_t gy_raw = imu_filter::le_i16(rx, 2);
    const int16_t gz_raw = imu_filter::le_i16(rx, 4);
    const int16_t ax_raw = imu_filter::le_i16(rx, 6);
    const int16_t ay_raw = imu_filter::le_i16(rx, 8);
    const int16_t az_raw = imu_filter::le_i16(rx, 10);

    const double ax = ax_raw * imu_filter::ACC_LSB_TO_MS2;
    const double ay = ay_raw * imu_filter::ACC_LSB_TO_MS2;
    const double az = az_raw * imu_filter::ACC_LSB_TO_MS2;

    const double gx = gx_raw * imu_filter::GYRO_LSB_TO_RAD_S;
    const double gy = gy_raw * imu_filter::GYRO_LSB_TO_RAD_S;
    const double gz = gz_raw * imu_filter::GYRO_LSB_TO_RAD_S;

    const auto state = imu_filter_.step(ax, ay, az, gx, gy, gz, dt);

    auto & data = imu_msg_.data;
    data[0] = state[0] + imu_filter::ROLL_OFFSET;
    data[1] = state[1] + imu_filter::PITCH_OFFSET;
    data[2] = state[2];
    data[3] = state[3];
    data[4] = state[4];
    data[5] = state[5];
    data[6] = state[6];
    data[7] = state[7];

    {
      std::lock_guard<std::mutex> guard(imu_state_lock_);
      imu_state_[0] = data[0];
      imu_state_[1] = data[1];
      imu_state_[2] = data[2];
      imu_state_[3] = data[3];
      imu_state_[4] = data[4];
      imu_state_[5] = data[5];
      imu_state_[6] = data[6];
      imu_state_[7] = data[7];
    }

    if (publish_compat_topics_ && imu_pub_) {
      imu_pub_->publish(imu_msg_);
    }
  }

  void encoder_set_directions(const std::array<int32_t, wheel_velocity::WHEEL_COUNT> & directions)
  {
    std::lock_guard<std::mutex> guard(encoder_lock_);
    for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
      encoder_direction_[i] = static_cast<int>(directions[i]);
    }
  }

  void encoder_callback(size_t motor_idx, uint32_t tick)
  {
    std::lock_guard<std::mutex> guard(encoder_lock_);
    const int dir_i = encoder_direction_[motor_idx];
    if (dir_i == 0) {
      return;
    }

    encoder_position_ticks_[motor_idx] += dir_i;
    encoder_last_edge_time_[motor_idx] = std::chrono::steady_clock::now();

    if (!encoder_last_edge_tick_valid_[motor_idx]) {
      encoder_last_edge_tick_us_[motor_idx] = tick;
      encoder_last_edge_tick_valid_[motor_idx] = true;
      return;
    }

    const uint32_t dt_us = encoder_odom::tick_diff(encoder_last_edge_tick_us_[motor_idx], tick);
    encoder_last_edge_tick_us_[motor_idx] = tick;

    if (dt_us == 0) {
      return;
    }

    const double period = dt_us * 1e-6;
    if (period < encoder_odom::MIN_VALID_PERIOD_S || period > encoder_odom::MAX_VALID_PERIOD_S) {
      return;
    }

    encoder_period_hist_[motor_idx].push_back(period);
    if (encoder_period_hist_[motor_idx].size() > 3) {
      encoder_period_hist_[motor_idx].pop_front();
    }

    const double period_med = encoder_odom::median(encoder_period_hist_[motor_idx]);
    const double omega_est = encoder_odom::TWO_PI / (encoder_ticks_per_rev_ * period_med);
    const double alpha = omega_est > encoder_odom::HIGH_SPEED_THRESHOLD ?
      encoder_odom::PERIOD_EMA_ALPHA_HIGH : encoder_odom::PERIOD_EMA_ALPHA_LOW;

    if (!encoder_period_ema_init_[motor_idx]) {
      encoder_period_ema_[motor_idx] = period_med;
      encoder_period_ema_init_[motor_idx] = true;
    } else {
      encoder_period_ema_[motor_idx] =
        alpha * period_med + (1.0 - alpha) * encoder_period_ema_[motor_idx];
    }

    const double omega =
      encoder_odom::TWO_PI / (encoder_ticks_per_rev_ * encoder_period_ema_[motor_idx]);
    encoder_omega_filtered_[motor_idx] = dir_i * omega;
  }

  void encoder_publish_state()
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - encoder_prev_time_).count();
    encoder_prev_time_ = now;

    std::array<int64_t, 3> ticks{};
    std::array<double, 3> omega{};
    std::array<std::chrono::steady_clock::time_point, 3> last_edge_time{};
    {
      std::lock_guard<std::mutex> guard(encoder_lock_);
      ticks = encoder_position_ticks_;
      omega = encoder_omega_filtered_;
      last_edge_time = encoder_last_edge_time_;
    }

    const auto omega_before_loop = omega;
    for (size_t i = 0; i < 3; ++i) {
      if (std::chrono::duration<double>(now - last_edge_time[i]).count() >
        encoder_odom::STOP_TIMEOUT)
      {
        omega[i] = 0.0;
        continue;
      }

      const int64_t delta_ticks = ticks[i] - encoder_prev_ticks_[i];
      encoder_dt_accum_[i] += dt;
      encoder_tick_accum_[i] += delta_ticks;

      if (encoder_dt_accum_[i] >= encoder_odom::DT_ACCUM_MIN) {
        const double omega_dt =
          (encoder_tick_accum_[i] / encoder_ticks_per_rev_) * encoder_odom::TWO_PI /
          encoder_dt_accum_[i];

        encoder_dt_accum_[i] = 0.0;
        encoder_tick_accum_[i] = 0;

        if (std::abs(omega[i]) > encoder_odom::HIGH_SPEED_THRESHOLD) {
          omega[i] = 0.5 * omega[i] + 0.5 * omega_dt;
        }
      }

      const double max_step = encoder_odom::OMEGA_RATE_LIMIT * dt;
      const double diff = omega[i] - omega_before_loop[i];

      if (std::abs(diff) > max_step) {
        omega[i] = omega_before_loop[i] + std::copysign(max_step, diff);
      }

      omega[i] = 0.8 * omega[i] + 0.2 * omega_before_loop[i];
    }

    encoder_prev_ticks_ = ticks;
    {
      std::lock_guard<std::mutex> guard(encoder_lock_);
      encoder_omega_filtered_ = omega;
    }

    auto & data = encoder_msg_.data;
    const double now_s = get_clock()->now().nanoseconds() * 1e-9;
    data[0] = static_cast<double>(now_s);

    for (size_t i = 0; i < 3; ++i) {
      const double rev = ticks[i] / encoder_ticks_per_rev_;
      const double angle = rev * encoder_odom::TWO_PI;
      const double distance = rev * encoder_wheel_circ_;
      const double linear_vel = omega[i] * encoder_wheel_circ_ / encoder_odom::TWO_PI;

      const size_t base = 1 + i * 5;
      data[base + 0] = static_cast<double>(ticks[i]);
      data[base + 1] = angle;
      data[base + 2] = distance;
      data[base + 3] = omega[i];
      data[base + 4] = linear_vel;
    }

    wheel_state_update(encoder_msg_);

    if (publish_compat_topics_ && wheel_state_pub_ && wheel1_speed_pub_) {
      std_msgs::msg::Float64 speed_msg;
      speed_msg.data = static_cast<double>(omega[0]);
      wheel1_speed_pub_->publish(speed_msg);
      wheel_state_pub_->publish(encoder_msg_);
    }
  }

  void wheel_ref_update(const std::array<double, wheel_velocity::WHEEL_COUNT> & refs)
  {
    std::lock_guard<std::mutex> guard(wheel_lock_);
    wheel_ref_vel_ = refs;
  }

  int wheel_apply_motor(size_t i, double duty, int direction)
  {
    const int duty_int = static_cast<int>(
      wheel_velocity::clamp(duty, 0.0, wheel_velocity::PWM_MAX));

    if (pi_ < 0) {
      return duty_int;
    }

    if (direction > 0) {
      set_PWM_dutycycle(pi_, wheel_velocity::PIN_LPWM[i], 0);
      set_PWM_dutycycle(pi_, wheel_velocity::PIN_RPWM[i], duty_int);
    } else if (direction < 0) {
      set_PWM_dutycycle(pi_, wheel_velocity::PIN_RPWM[i], 0);
      set_PWM_dutycycle(pi_, wheel_velocity::PIN_LPWM[i], duty_int);
    } else {
      set_PWM_dutycycle(pi_, wheel_velocity::PIN_RPWM[i], 0);
      set_PWM_dutycycle(pi_, wheel_velocity::PIN_LPWM[i], 0);
    }

    return duty_int;
  }

  void wheel_stop_all()
  {
    std::array<int32_t, wheel_velocity::WHEEL_COUNT> directions{{0, 0, 0}};

    {
      std::lock_guard<std::mutex> guard(wheel_lock_);
      for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
        wheel_apply_motor(i, 0.0, 0);
        wheel_pi_ctrl_[i].reset();
        wheel_last_direction_[i] = 0;
      }

      for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
        dir_msg_.data[i] = directions[i];
      }
    }

    encoder_set_directions(directions);

    if (publish_compat_topics_ && dir_pub_) {
      dir_pub_->publish(dir_msg_);
    }
  }

  void wheel_state_update(const std_msgs::msg::Float64MultiArray & msg)
  {
    if (msg.data.size() < 16) {
      return;
    }

    std::array<int32_t, wheel_velocity::WHEEL_COUNT> dir_out{{0, 0, 0}};

    {
      std::lock_guard<std::mutex> guard(wheel_lock_);

      for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
        wheel_meas_vel_[i] =
          static_cast<double>(msg.data[wheel_velocity::WHEEL_STATE_ANGULAR_VEL_IDXS[i]]);
      }

      const auto now = get_clock()->now();
      double dt = (now - wheel_last_time_).nanoseconds() * 1e-9;
      wheel_last_time_ = now;
      dt = wheel_velocity::clamp(dt, wheel_velocity::DT_MIN, wheel_velocity::DT_MAX);

      for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
        const double ref = wheel_ref_vel_[i];
        const double meas = wheel_meas_vel_[i];

        if (std::abs(ref) < wheel_velocity::REF_DEADBAND_OMEGA) {
          wheel_pi_ctrl_[i].reset();
          wheel_apply_motor(i, 0.0, 0);
          dir_out[i] = 0;
          continue;
        }

        const int direction = ref > 0.0 ? 1 : -1;

        if (direction != wheel_last_direction_[i] && wheel_last_direction_[i] != 0) {
          wheel_pi_ctrl_[i].reset();
        }

        wheel_last_direction_[i] = direction;

        const double ref_abs = std::abs(ref);
        const double meas_abs = std::abs(meas);

        const auto [u, error] = wheel_pi_ctrl_[i].update(ref_abs, meas_abs, dt);
        (void)error;

        double duty = wheel_velocity::clamp(u, 0.0, wheel_velocity::PWM_MAX);
        if (duty > 0.0) {
          duty = std::max(wheel_velocity::PWM_START_MOVE, duty);
        }

        dir_out[i] = direction;
        wheel_apply_motor(i, duty, direction);
      }

      for (size_t i = 0; i < wheel_velocity::WHEEL_COUNT; ++i) {
        dir_msg_.data[i] = dir_out[i];
      }
    }

    encoder_set_directions(dir_out);

    if (publish_compat_topics_ && dir_pub_) {
      dir_pub_->publish(dir_msg_);
    }
  }

  void pid_print_status()
  {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    {
      std::lock_guard<std::mutex> guard(pid_gain_lock_);
      kp = pid_kp_;
      ki = pid_ki_;
      kd = pid_kd_;
    }

    RCLCPP_INFO(get_logger(), "ACTUAL: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
  }

  void keyboard_loop()
  {
    balance_pid::TerminalState terminal;
    bool configured = false;

    try {
      terminal = balance_pid::setup_terminal();
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
          std::lock_guard<std::mutex> guard(pid_gain_lock_);
          if (ch == 'q') {
            pid_kp_ += balance_pid::KP_STEP;
          } else if (ch == 'a') {
            pid_kp_ -= balance_pid::KP_STEP;
          } else if (ch == 'w') {
            pid_ki_ += balance_pid::KI_STEP;
          } else if (ch == 's') {
            pid_ki_ -= balance_pid::KI_STEP;
          } else if (ch == 'e') {
            pid_kd_ += balance_pid::KD_STEP;
          } else if (ch == 'd') {
            pid_kd_ -= balance_pid::KD_STEP;
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
          pid_print_status();
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Keyboard thread error: %s", error.what());
    }

    if (configured) {
      balance_pid::restore_terminal(terminal);
    }
  }

  double pid_deadband(double x, double threshold)
  {
    if (std::abs(x) < threshold) {
      return 0.0;
    }
    return x;
  }

  double pid_min_command_filter(double x)
  {
    if (std::abs(x) < balance_pid::MIN_COMMAND_RAD) {
      return 0.0;
    }
    return x;
  }

  std::tuple<double, double, double, double> pid_limit_wheels(double w1, double w2, double w3)
  {
    const double max_w = std::max({std::abs(w1), std::abs(w2), std::abs(w3)});
    double scale = 1.0;

    if (max_w > balance_pid::MAX_W_RAD) {
      scale = balance_pid::MAX_W_RAD / max_w;
      w1 *= scale;
      w2 *= scale;
      w3 *= scale;
    }

    return {w1, w2, w3, scale};
  }

  void pid_control_loop()
  {
    const auto now = get_clock()->now();
    double dt = (now - pid_last_time_).nanoseconds() * 1e-9;
    pid_last_time_ = now;

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
      std::lock_guard<std::mutex> guard(pid_gain_lock_);
      kp = pid_kp_;
      ki = pid_ki_;
      kd = pid_kd_;
    }

    double roll = 0.0;
    double pitch = 0.0;
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    {
      std::lock_guard<std::mutex> guard(imu_state_lock_);
      roll = imu_state_[0];
      pitch = imu_state_[1];
      roll_rate = imu_state_[2];
      pitch_rate = imu_state_[3];
    }

    const double theta_x = pid_deadband(pitch, balance_pid::ANGLE_DEADBAND);
    const double theta_y = pid_deadband(roll, balance_pid::ANGLE_DEADBAND);
    const double theta_dot_x = pitch_rate;
    const double theta_dot_y = roll_rate;

    if (theta_x == 0.0 && theta_y == 0.0) {
      pid_x_.reset();
      pid_y_.reset();

      pid_cmd_vel_x_ = 0.0;
      pid_cmd_vel_y_ = 0.0;
    } else {
      pid_cmd_vel_x_ = pid_x_.update(theta_x, theta_dot_x, dt, kp, ki, kd);
      pid_cmd_vel_y_ = pid_y_.update(theta_y, theta_dot_y, dt, kp, ki, kd);
    }

    double vx_r = 0.70710678 * pid_cmd_vel_x_ - 0.70710678 * pid_cmd_vel_y_;
    double vy_r = 0.70710678 * pid_cmd_vel_x_ + 0.70710678 * pid_cmd_vel_y_;

    double V1 = -vy_r * std::cos(balance_pid::PI_VALUE / 4.0);
    double V2 = (-balance_pid::SQRT3_2 * vx_r + 0.5 * vy_r) *
      std::cos(balance_pid::PI_VALUE / 4.0);
    double V3 = (balance_pid::SQRT3_2 * vx_r + 0.5 * vy_r) *
      std::cos(balance_pid::PI_VALUE / 4.0);

    double w1 = V1 / balance_pid::r_k;
    double w2 = V2 / balance_pid::r_k;
    double w3 = V3 / balance_pid::r_k;

    double wheel_scale = 1.0;
    std::tie(w1, w2, w3, wheel_scale) = pid_limit_wheels(w1, w2, w3);

    if (wheel_scale < 1.0) {
      pid_cmd_vel_x_ *= wheel_scale;
      pid_cmd_vel_y_ *= wheel_scale;
      vx_r *= wheel_scale;
      vy_r *= wheel_scale;

      pid_x_.anti_windup(pid_cmd_vel_x_, kp, ki, kd);
      pid_y_.anti_windup(pid_cmd_vel_y_, kp, ki, kd);
    }

    w1 = pid_min_command_filter(w1);
    w2 = pid_min_command_filter(w2);
    w3 = pid_min_command_filter(w3);

    std::array<double, wheel_velocity::WHEEL_COUNT> refs{{
      static_cast<double>(-w3),
      static_cast<double>(-w2),
      static_cast<double>(-w1)}};

    wheel_ref_update(refs);

    pid_msg_.data[0] = refs[0];
    pid_msg_.data[1] = refs[1];
    pid_msg_.data[2] = refs[2];

    if (publish_compat_topics_ && vel_pub_) {
      vel_pub_->publish(pid_msg_);
    }

    pid_log_counter_ += 1;
    if (pid_log_counter_ >= 100) {
      pid_log_counter_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "pitch=%.4f roll=%.4f cmd_x=%.4f cmd_y=%.4f vx=%.4f vy=%.4f w1=%.2f w2=%.2f w3=%.2f",
        pitch, roll, pid_cmd_vel_x_, pid_cmd_vel_y_, vx_r, vy_r, w1, w2, w3);
    }
  }

  bool publish_compat_topics_{true};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel1_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr dir_pub_;

  rclcpp::CallbackGroup::SharedPtr encoder_group_;
  rclcpp::CallbackGroup::SharedPtr pid_group_;
  rclcpp::TimerBase::SharedPtr encoder_timer_;
  rclcpp::TimerBase::SharedPtr pid_timer_;

  int pi_{-1};
  std::atomic<bool> cleaned_up_{false};

  imu_filter::I2cDevice imu_i2c_;
  imu_filter::TiltEkf imu_filter_;
  std::chrono::steady_clock::time_point imu_last_time_;
  std::atomic<bool> imu_stop_{false};
  std::thread imu_thread_;
  std::mutex imu_state_lock_;
  std::array<double, 8> imu_state_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std_msgs::msg::Float64MultiArray imu_msg_;

  std::array<unsigned, 3> encoder_pins_{
    encoder_odom::ENCODER_PIN_1A,
    encoder_odom::ENCODER_PIN_2A,
    encoder_odom::ENCODER_PIN_3A};
  std::array<int, 3> encoder_callbacks_{{-1, -1, -1}};
  std::array<EncoderCallbackContext, 3> encoder_callback_contexts_{};

  std::mutex encoder_lock_;
  std::array<int, 3> encoder_direction_{{0, 0, 0}};
  std::array<int64_t, 3> encoder_position_ticks_{{0, 0, 0}};
  std::array<uint32_t, 3> encoder_last_edge_tick_us_{{0, 0, 0}};
  std::array<bool, 3> encoder_last_edge_tick_valid_{{false, false, false}};
  std::array<std::chrono::steady_clock::time_point, 3> encoder_last_edge_time_{};
  std::array<std::deque<double>, 3> encoder_period_hist_{};
  std::array<double, 3> encoder_period_ema_{{0.0, 0.0, 0.0}};
  std::array<bool, 3> encoder_period_ema_init_{{false, false, false}};
  std::array<int64_t, 3> encoder_prev_ticks_{{0, 0, 0}};
  std::chrono::steady_clock::time_point encoder_prev_time_{std::chrono::steady_clock::now()};
  std::array<double, 3> encoder_dt_accum_{{0.0, 0.0, 0.0}};
  std::array<int64_t, 3> encoder_tick_accum_{{0, 0, 0}};
  std::array<double, 3> encoder_omega_filtered_{{0.0, 0.0, 0.0}};
  double encoder_ticks_per_rev_{static_cast<double>(encoder_odom::PPR)};
  double encoder_wheel_circ_{encoder_odom::PI_VALUE * encoder_odom::WHEEL_DIAMETER};
  std_msgs::msg::Float64MultiArray encoder_msg_;

  std::mutex wheel_lock_;
  std::array<double, wheel_velocity::WHEEL_COUNT> wheel_ref_vel_{{0.0, 0.0, 0.0}};
  std::array<double, wheel_velocity::WHEEL_COUNT> wheel_meas_vel_{{0.0, 0.0, 0.0}};
  rclcpp::Time wheel_last_time_;
  std::array<int, wheel_velocity::WHEEL_COUNT> wheel_last_direction_{{0, 0, 0}};
  std::array<wheel_velocity::PIController, wheel_velocity::WHEEL_COUNT> wheel_pi_ctrl_{{
    wheel_velocity::PIController(3.5, 23.0, 120.0),
    wheel_velocity::PIController(3.5, 23.0, 120.0),
    wheel_velocity::PIController(3.5, 23.0, 120.0)}};
  std_msgs::msg::Int32MultiArray dir_msg_;

  double pid_cmd_vel_x_{0.0};
  double pid_cmd_vel_y_{0.0};
  double pid_kp_{balance_pid::KP};
  double pid_ki_{balance_pid::KI};
  double pid_kd_{balance_pid::KD};
  std::mutex pid_gain_lock_;
  balance_pid::AnglePid pid_x_;
  balance_pid::AnglePid pid_y_;
  rclcpp::Time pid_last_time_;
  int pid_log_counter_{0};
  std_msgs::msg::Float64MultiArray pid_msg_;
  std::atomic<bool> keyboard_stop_{false};
  std::thread keyboard_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<BallbotCombinedNode> node;
  try {
    node = std::make_shared<BallbotCombinedNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ballbot_combined"), "%s", error.what());
  }

  if (node) {
    node->cleanup();
    node.reset();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
