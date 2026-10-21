// Copyright 2026 wee733
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "c20d_driver/msg/angle_state.hpp"
#include "c20d_driver/msg/angular_rate_command.hpp"
#include "c20d_driver/msg/angular_rate_state.hpp"
#include "c20d_driver/msg/attitude_command.hpp"
#include "c20d_driver/msg/camera_data.hpp"
#include "c20d_driver/msg/carrier_data.hpp"
#include "c20d_driver/msg/gimbal_state.hpp"
#include "c20d_driver/msg/proportional_rate_command.hpp"
#include "c20d_driver/msg/relative_angle_command.hpp"
#include "c20d_driver/protocol.hpp"
#include "c20d_driver/serial_port.hpp"
#include "c20d_driver/srv/device_command.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace c20d_driver
{
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;

double age(Time time)
{
  return std::chrono::duration<double>(Clock::now() - time).count();
}

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("c20d_driver")
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    port_name_ = declare_parameter("port", "/dev/ttyTHS1", fixed);
    frame_id_ = declare_parameter("frame_id", "c20d_head_up", fixed);
    const double rate = declare_parameter("update_rate", 100.0, fixed);
    feedback_timeout_ = declare_parameter("feedback_timeout", 0.5, fixed);
    command_timeout_ = declare_parameter("command_timeout", 0.5, fixed);
    auxiliary_timeout_ = declare_parameter("auxiliary_timeout", 0.5, fixed);
    max_rate_ = declare_parameter("max_rate", 0.5 / kDegToRad, fixed);
    roll_limit_ = declare_parameter("roll_limit", 60.0, fixed);
    pitch_limit_ = declare_parameter("pitch_limit", 120.0, fixed);
    control_enabled_ = declare_parameter("control_enabled", true, fixed);
    const std::array<double, 7> parameters = {rate, feedback_timeout_, command_timeout_,
      auxiliary_timeout_, max_rate_, roll_limit_, pitch_limit_};
    for (auto value : parameters) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Timing and limit parameters must be finite and positive");
      }
    }
    if (rate < 50 || rate > 150 || max_rate_ > 1500.0 ||
      roll_limit_ > 60.0 || pitch_limit_ > 120.0)
    {
      throw std::invalid_argument("Rate must be 50..150 Hz; limits cannot exceed horizontal C20D");
    }
    state_pub_ = create_publisher<msg::GimbalState>("~/state", rclcpp::SensorDataQoS());
    relative_pub_ = create_publisher<msg::AngleState>(
      "~/state/relative_angle", rclcpp::SensorDataQoS());
    attitude_pub_ = create_publisher<msg::AngleState>(
      "~/state/attitude", rclcpp::SensorDataQoS());
    rate_pub_ = create_publisher<msg::AngularRateState>(
      "~/state/angular_rate", rclcpp::SensorDataQoS());
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_states", rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    // Advertise schemas even before the first command; retain only accepted inputs.
    const auto echo_qos = rclcpp::QoS(1).reliable().transient_local();
    relative_echo_pub_ = create_publisher<msg::RelativeAngleCommand>(
      "~/command_echo/relative_angle", echo_qos);
    attitude_echo_pub_ = create_publisher<msg::AttitudeCommand>(
      "~/command_echo/attitude", echo_qos);
    rate_echo_pub_ = create_publisher<msg::AngularRateCommand>(
      "~/command_echo/angular_rate", echo_qos);
    proportional_echo_pub_ = create_publisher<msg::ProportionalRateCommand>(
      "~/command_echo/proportional_rate", echo_qos);
    auto command_qos = rclcpp::QoS(1).reliable().durability_volatile();
    command_qos.lifespan(rclcpp::Duration::from_seconds(command_timeout_));
    relative_sub_ = create_subscription<msg::RelativeAngleCommand>(
      "~/command/relative_angle", command_qos,
      [this](const msg::RelativeAngleCommand & m) {
        if (manual(m.header, m.roll_deg, m.pitch_deg, 2, 0,
          m.fpv_sensitivity, m.center_roll, m.center_pitch))
        {
          relative_echo_pub_->publish(m);
        }
      });
    attitude_sub_ = create_subscription<msg::AttitudeCommand>(
      "~/command/attitude", command_qos,
      [this](const msg::AttitudeCommand & m) {
        if (manual(m.header, m.roll_deg, m.pitch_deg, 1, 0, 0, m.center_roll, m.center_pitch)) {
          attitude_echo_pub_->publish(m);
        }
      });
    rate_sub_ = create_subscription<msg::AngularRateCommand>(
      "~/command/angular_rate", command_qos,
      [this](const msg::AngularRateCommand & m) {
        if (manual(m.header, m.roll_deg_s, m.pitch_deg_s, 1, 2)) {
          rate_echo_pub_->publish(m);
        }
      });
    proportional_sub_ = create_subscription<msg::ProportionalRateCommand>(
      "~/command/proportional_rate", command_qos,
      [this](const msg::ProportionalRateCommand & m) {
        if (manual(m.header, m.roll_deg_s, m.pitch_deg_s, 1, 1)) {
          proportional_echo_pub_->publish(m);
        }
      });
    carrier_sub_ = create_subscription<msg::CarrierData>(
      "~/carrier", rclcpp::QoS(1), [this](const msg::CarrierData & m) {carrier(m);});
    camera_sub_ = create_subscription<msg::CameraData>(
      "~/camera", rclcpp::QoS(1), [this](const msg::CameraData & m) {camera(m);});
    service_ = create_service<srv::DeviceCommand>(
      "~/device_command",
      [this](const std::shared_ptr<srv::DeviceCommand::Request> request,
      std::shared_ptr<srv::DeviceCommand::Response> response)
      {
        try {
          require_link();
          const auto code = request->command;
          if (code < 1 || code > 6) {
            throw std::invalid_argument("Command must be 1..6");
          }
          if (code == 1 && !last_feedback_.temperature_ready) {
            throw std::runtime_error("Gyro calibration requires temperature_ready");
          }
          if ((code == 5 || code == 6) && age(camera_time_) > auxiliary_timeout_) {
            throw std::runtime_error("Point/track requires fresh camera data");
          }
          if (code == 4) {
            hold();
          }
          queue_command(code);
          active_mode_ = code;
          response->accepted = true;
          response->message = "Queued; execution result is reported on state";
        } catch (const std::exception & e) {
          response->accepted = false;
          response->message = e.what();
        }
      });
    timer_ = create_wall_timer(
      std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate)), [this]() {tick();});
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {diagnostics();});
    RCLCPP_INFO(get_logger(), "C20D horizontal, %s, 115200 8N1, %.0f Hz", port_name_.c_str(), rate);
  }

  ~DriverNode() override
  {
    // Best effort zero-rate hold, never disable stabilization on shutdown.
    if (serial_.is_open() && active_mode_ == 4 && online()) {
      hold();
      control_.command = 0;
      try {
        const auto frame = encode(control_);
        serial_.write(frame.data(), frame.size());
      } catch (const std::exception &) {
        // Hardware link may already be disconnected.
      }
    }
  }

private:
  bool online() const
  {
    return serial_.is_open() && received_ && age(feedback_time_) < feedback_timeout_;
  }

  void require_link() const
  {
    if (!control_enabled_) {
      throw std::runtime_error("Control disabled by parameter");
    }
    if (!online()) {
      throw std::runtime_error("No fresh CRC-valid device feedback");
    }
    if (last_feedback_.hardware_error != 0) {
      throw std::runtime_error("Device reports a hardware fault");
    }
    if (control_.command != 0) {
      throw std::runtime_error("Previous command awaiting device reply");
    }
  }

  void check_stamp(const std_msgs::msg::Header & header, double timeout) const
  {
    if (header.stamp.sec == 0 && header.stamp.nanosec == 0) {
      return;
    }
    const double delta = (now() - rclcpp::Time(header.stamp,
        get_clock()->get_clock_type())).seconds();
    if (delta > timeout || delta < -0.1) {
      throw std::invalid_argument("Message timestamp is stale or in the future");
    }
  }

  void queue_command(uint8_t command)
  {
    control_.trigger = (control_.trigger + 1) & 7;
    control_.command = command;
    pending_time_ = Clock::now();
    pending_sent_ = false;
  }

  void hold()
  {
    for (auto & axis : control_.axes) {
      axis.mode = 1;
      axis.type = 2;
      axis.value = 0;
    }
    rate_active_ = false;
  }

  bool manual(
    const std_msgs::msg::Header & header, double roll, double pitch, uint8_t mode, uint8_t type,
    int8_t sensitivity = 0, bool center_roll = false, bool center_pitch = false)
  {
    try {
      require_link();
      check_stamp(header, command_timeout_);
      if (last_feedback_.state == 1 || last_feedback_.state == 2 || last_feedback_.state == 3) {
        throw std::runtime_error("Device initializing, stopped or protected");
      }
      const std::array<double, 2> values{roll, pitch};
      const std::array<double, 2> limits{roll_limit_, pitch_limit_};
      const std::array<bool, 2> center{center_roll, center_pitch};
      if (sensitivity < -16 || sensitivity > 15) {
        throw std::invalid_argument("Invalid FPV sensitivity");
      }
      auto candidate = control_;
      bool rate_active = false;
      for (std::size_t i = 0; i < 2; ++i) {
        if (type == 1 && (age(camera_time_) > auxiliary_timeout_ ||
          control_.zoom == 0 || control_.vertical_fov == 0))
        {
          throw std::invalid_argument("Proportional rate requires fresh nonzero zoom/FOV");
        }
        if ((mode != 1 && mode != 2) || type > 2 ||
          (mode == 2 && type != 0) || !std::isfinite(values[i]) ||
          std::abs(values[i]) > (type == 0 ? limits[i] : max_rate_))
        {
          throw std::invalid_argument("Invalid axis mode, non-finite value or exceeded limit");
        }
        auto & a = candidate.axes[i];
        a.mode = mode;
        a.type = type;
        // Tested upright C20D: all wire pitch targets are positive head-up.
        // Relative and IMU rate feedback have the opposite sign (see publish()).
        a.value = quantize(values[i], type == 0 ? 0.01 : 0.1);
        if (center[i]) {
          a.center = !a.center;
        }
        rate_active = rate_active || type != 0;
      }
      // Horizontal C20D has no independently commanded yaw motor.
      candidate.axes[2].mode = mode == 2 ? 2 : 1;
      candidate.axes[2].type = mode == 2 ? 0 : 2;
      candidate.axes[2].value = 0;
      candidate.sensitivity = sensitivity;
      control_ = candidate;
      command_time_ = Clock::now();
      rate_active_ = rate_active;
      if (active_mode_ != 4) {
        queue_command(4);
        active_mode_ = 4;
      }
      return true;
    } catch (const std::exception & e) {
      ++rejected_commands_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected command: %s", e.what());
      return false;
    }
  }

  void carrier(const msg::CarrierData & m)
  {
    try {
      check_stamp(m.header, auxiliary_timeout_);
      auto candidate = control_;
      candidate.carrier_valid = m.valid;
      if (m.valid) {
        for (std::size_t i = 0; i < 3; ++i) {
          const auto angle = m.attitude_rpy[i];
          const auto limit = i == 1 ? 90.0 : 180.0;
          if (!std::isfinite(angle) || angle < -limit || angle > limit ||
            (i != 1 && angle == limit))
          {
            throw std::invalid_argument("Carrier attitude out of range");
          }
          candidate.carrier_angle[i] = quantize(angle, 0.01);
          if (i != 1 && candidate.carrier_angle[i] == 18000) {
            candidate.carrier_angle[i] = -18000;
          }
          candidate.carrier_accel[i] = quantize(m.acceleration_neu[i], 0.01);
        }
      }
      control_ = candidate;
      carrier_time_ = Clock::now();
    } catch (const std::exception & e) {
      control_.carrier_valid = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected carrier: %s", e.what());
    }
  }

  void camera(const msg::CameraData & m)
  {
    try {
      check_stamp(m.header, auxiliary_timeout_);
      if (m.vertical_fov_deg > 127 || !std::isfinite(m.zoom) || m.zoom < 0 ||
        m.zoom > 16777.215 || !std::isfinite(m.horizontal_offset) ||
        !std::isfinite(m.vertical_offset) || std::abs(m.horizontal_offset) > 180.0 ||
        std::abs(m.vertical_offset) > 180.0)
      {
        throw std::invalid_argument("Invalid camera data");
      }
      control_.vertical_fov = m.vertical_fov_deg;
      control_.zoom = static_cast<uint32_t>(std::llround(m.zoom * 1000));
      control_.target_angle = {
        static_cast<float>(m.horizontal_offset),
        static_cast<float>(m.vertical_offset)};
      camera_time_ = Clock::now();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected camera: %s", e.what());
    }
  }

  void publish(const Feedback & f)
  {
    msg::GimbalState m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.firmware_version_raw = f.firmware;
    m.hardware_error = f.hardware_error;
    m.upright = f.upright;
    m.gimbal_state = f.state;
    m.temperature_ready = f.temperature_ready;
    m.command_status = f.command_status;
    m.command_code = f.command;
    m.camera_rate_raw_pyr = f.rate;
    m.camera_attitude_raw_rpy = f.attitude;
    m.relative_angle_raw_pyr = f.relative;
    m.raw_frame = f.raw;
    const std::array<std::size_t, 3> reorder{1, 0, 2};
    for (std::size_t i = 0; i < 3; ++i) {
      m.camera_rate_rpy[i] = f.rate[reorder[i]] * 0.1;
      m.camera_attitude_rpy[i] = f.attitude[i] * 0.01;
      m.relative_angle_rpy[i] = f.relative[reorder[i]] * 0.01;
    }
    // Head-up user convention; raw arrays and raw_frame remain lossless vendor data.
    m.relative_angle_rpy[1] = -m.relative_angle_rpy[1];
    m.camera_rate_rpy[1] = -m.camera_rate_rpy[1];
    state_pub_->publish(m);
    msg::AngleState relative;
    relative.header = m.header;
    relative.roll_deg = m.relative_angle_rpy[0];
    relative.pitch_deg = m.relative_angle_rpy[1];
    relative.yaw_deg = m.relative_angle_rpy[2];
    relative_pub_->publish(relative);
    msg::AngleState attitude;
    attitude.header = m.header;
    attitude.roll_deg = m.camera_attitude_rpy[0];
    attitude.pitch_deg = m.camera_attitude_rpy[1];
    attitude.yaw_deg = m.camera_attitude_rpy[2];
    attitude_pub_->publish(attitude);
    msg::AngularRateState rate;
    rate.header = m.header;
    rate.roll_deg_s = m.camera_rate_rpy[0];
    rate.pitch_deg_s = m.camera_rate_rpy[1];
    rate.yaw_deg_s = m.camera_rate_rpy[2];
    rate_pub_->publish(rate);
    sensor_msgs::msg::JointState joints;
    joints.header = m.header;
    joints.name = {"c20d_roll_joint", "c20d_pitch_joint"};
    // Standard JointState positions must remain radians. Custom messages use degrees.
    joints.position = {
      m.relative_angle_rpy[0] * kDegToRad, m.relative_angle_rpy[1] * kDegToRad};
    // IMU body rates are not joint velocities; leave velocity/effort absent.
    joints_pub_->publish(joints);
  }

  void receive()
  {
    std::array<uint8_t, 1024> buffer{};
    for (int batch = 0; batch < 8; ++batch) {
      const auto count = serial_.read(buffer.data(), buffer.size());
      if (count == 0) {
        break;
      }
      for (const auto & f : parser_.feed(buffer.data(), count)) {
        if (!received_) {
          RCLCPP_INFO(get_logger(), "Feedback acquired: firmware_raw=%u, hw_error=%u",
            f.firmware, f.hardware_error);
        }
        received_ = true;
        feedback_time_ = Clock::now();
        last_feedback_ = f;
        ++frames_;
        if (pending_sent_ && control_.command != 0 && f.command == control_.command) {
          if (f.command_status == 2) {
            RCLCPP_ERROR(get_logger(), "Device rejected command %u", f.command);
            active_mode_ = 0;
            hold();
          }
          control_.command = 0;  // Official requirement: clear on receipt of reply.
          pending_sent_ = false;
        }
        publish(f);
      }
    }
  }

  void tick()
  {
    if (!serial_.is_open()) {
      if (age(reconnect_time_) < 1.0) {
        return;
      }
      reconnect_time_ = Clock::now();
      try {
        serial_.open(port_name_);
        control_ = Control{};
        active_mode_ = 0;
        pending_sent_ = false;
        rate_active_ = false;
        received_ = false;
        camera_time_ = Time{};
        carrier_time_ = Time{};
        parser_.reset();
        opened_time_ = Clock::now();
        RCLCPP_INFO(get_logger(), "UART opened; polling, waiting for feedback");
      } catch (const std::exception & e) {
        last_error_ = e.what();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        return;
      }
    }
    try {
      receive();
      if (age(received_ ? feedback_time_ : opened_time_) > feedback_timeout_) {
        throw std::runtime_error("Feedback timeout; reconnecting without replaying commands");
      }
      if (control_.command != 0 && age(pending_time_) > feedback_timeout_) {
        throw std::runtime_error("Device command acknowledgement timeout");
      }
      if (rate_active_ && age(command_time_) > command_timeout_) {
        for (auto & axis : control_.axes) {
          if (axis.type != 0) {
            axis.value = 0;
          }
        }
        rate_active_ = false;
        RCLCPP_WARN(get_logger(), "Rate command expired: commanded zero rate");
      }
      if ((active_mode_ == 5 || active_mode_ == 6) && age(camera_time_) > auxiliary_timeout_) {
        hold();
        queue_command(4);
        active_mode_ = 4;
        RCLCPP_WARN(get_logger(), "Camera target expired: switching to manual zero-rate hold");
      }
      if (age(carrier_time_) > auxiliary_timeout_) {
        control_.carrier_valid = false;
      }
      if (age(camera_time_) > auxiliary_timeout_) {
        for (auto & axis : control_.axes) {
          if (axis.type == 1) {
            axis.value = 0;
          }
        }
      }
      const auto frame = encode(control_);
      serial_.write(frame.data(), frame.size());
      pending_sent_ = control_.command != 0;
    } catch (const std::exception & e) {
      last_error_ = e.what();
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      serial_.close();
      received_ = false;
    }
  }

  void diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "c20d_driver/serial";
    status.hardware_id = port_name_;
    status.level = online() ? status.OK : status.ERROR;
    status.message = online() ? "Receiving CRC-valid feedback" : "Offline: " + last_error_;
    if (online() && last_feedback_.hardware_error != 0) {
      status.level = status.ERROR;
      status.message = "Device hardware fault";
    } else if (online() && last_feedback_.state == 3) {
      status.level = status.WARN;
      status.message = "Device tilt protection active";
    }
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(entry);
      };
    add("valid_frames", std::to_string(frames_));
    add("crc_errors", std::to_string(parser_.crc_errors()));
    add("discarded_bytes", std::to_string(parser_.discarded_bytes()));
    add("rejected_commands", std::to_string(rejected_commands_));
    add("firmware_raw", std::to_string(last_feedback_.firmware));
    add("hardware_error", std::to_string(last_feedback_.hardware_error));
    add("gimbal_state", std::to_string(last_feedback_.state));
    add("temperature_ready", last_feedback_.temperature_ready ? "true" : "false");
    add("feedback_age_s", received_ ? std::to_string(age(feedback_time_)) : "unavailable");
    add("pending_command", std::to_string(control_.command));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  SerialPort serial_;
  Parser parser_;
  Control control_;
  Feedback last_feedback_;
  std::string port_name_, frame_id_, last_error_;
  double feedback_timeout_, command_timeout_, auxiliary_timeout_, max_rate_, roll_limit_,
    pitch_limit_;
  bool control_enabled_ = true;
  bool received_ = false;
  bool pending_sent_ = false;
  bool rate_active_ = false;
  uint8_t active_mode_ = 0;
  uint64_t frames_ = 0;
  uint64_t rejected_commands_ = 0;
  Time opened_time_{}, feedback_time_{}, reconnect_time_{}, pending_time_{};
  Time command_time_{}, camera_time_{}, carrier_time_{};
  rclcpp::Publisher<msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<msg::AngleState>::SharedPtr relative_pub_, attitude_pub_;
  rclcpp::Publisher<msg::AngularRateState>::SharedPtr rate_pub_;
  rclcpp::Publisher<msg::RelativeAngleCommand>::SharedPtr relative_echo_pub_;
  rclcpp::Publisher<msg::AttitudeCommand>::SharedPtr attitude_echo_pub_;
  rclcpp::Publisher<msg::AngularRateCommand>::SharedPtr rate_echo_pub_;
  rclcpp::Publisher<msg::ProportionalRateCommand>::SharedPtr proportional_echo_pub_;
  rclcpp::Subscription<msg::RelativeAngleCommand>::SharedPtr relative_sub_;
  rclcpp::Subscription<msg::AttitudeCommand>::SharedPtr attitude_sub_;
  rclcpp::Subscription<msg::AngularRateCommand>::SharedPtr rate_sub_;
  rclcpp::Subscription<msg::ProportionalRateCommand>::SharedPtr proportional_sub_;
  rclcpp::Subscription<msg::CarrierData>::SharedPtr carrier_sub_;
  rclcpp::Subscription<msg::CameraData>::SharedPtr camera_sub_;
  rclcpp::Service<srv::DeviceCommand>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostics_timer_;
};
}  // namespace c20d_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<c20d_driver::DriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("c20d_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
