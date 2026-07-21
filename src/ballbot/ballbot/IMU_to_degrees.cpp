#include <chrono>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
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
}  // namespace

class ImuKalmanNode : public rclcpp::Node
{
public:
  ImuKalmanNode()
  : Node("imu_kalman_node"),
    i2c_("/dev/i2c-1", I2C_ADDRESS),
    last_time_(std::chrono::steady_clock::now())
  {
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("/imu/kalman_state", 1);
    initialize_lsm6dso32(i2c_);
    msg_.data.assign(8, 0.0);
  }

  void loop_once()
  {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;

    if (dt <= 0.0) {
      dt = 0.001;
    }

    uint8_t rx[BURST_LEN] = {};
    if (!i2c_.read_burst(BURST_START_REG, rx, BURST_LEN)) {
      return;
    }

    const int16_t gx_raw = le_i16(rx, 0);
    const int16_t gy_raw = le_i16(rx, 2);
    const int16_t gz_raw = le_i16(rx, 4);
    const int16_t ax_raw = le_i16(rx, 6);
    const int16_t ay_raw = le_i16(rx, 8);
    const int16_t az_raw = le_i16(rx, 10);

    const double ax = ax_raw * ACC_LSB_TO_MS2;
    const double ay = ay_raw * ACC_LSB_TO_MS2;
    const double az = az_raw * ACC_LSB_TO_MS2;

    const double gx = gx_raw * GYRO_LSB_TO_RAD_S;
    const double gy = gy_raw * GYRO_LSB_TO_RAD_S;
    const double gz = gz_raw * GYRO_LSB_TO_RAD_S;

    const auto state = filter_.step(ax, ay, az, gx, gy, gz, dt);
    auto & data = msg_.data;
    data[0] = state[0] + ROLL_OFFSET;
    data[1] = state[1] + PITCH_OFFSET;
    data[2] = state[2];
    data[3] = state[3];
    data[4] = state[4];
    data[5] = state[5];
    data[6] = state[6];
    data[7] = state[7];

    publisher_->publish(msg_);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  I2cDevice i2c_;
  TiltEkf filter_;
  std::chrono::steady_clock::time_point last_time_;
  std_msgs::msg::Float64MultiArray msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ImuKalmanNode>();
    while (rclcpp::ok()) {
      node->loop_once();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("IMU_to_degrees"), "%s", error.what());
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
