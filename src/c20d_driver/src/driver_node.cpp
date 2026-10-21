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

#include "c20d_driver/msg/camera_data.hpp"
#include "c20d_driver/msg/carrier_data.hpp"
#include "c20d_driver/msg/gimbal_command.hpp"
#include "c20d_driver/msg/gimbal_state.hpp"
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
    frame_id_ = declare_parameter("frame_id", "c20d_vendor", fixed);
    const double rate = declare_parameter("update_rate", 100.0, fixed);
    feedback_timeout_ = declare_parameter("feedback_timeout", 0.5, fixed);
    command_timeout_ = declare_parameter("command_timeout", 0.5, fixed);
    auxiliary_timeout_ = declare_parameter("auxiliary_timeout", 0.5, fixed);
    max_rate_ = declare_parameter("max_rate", 0.5 / kDegToRad, fixed);
    roll_limit_ = declare_parameter("roll_limit", 60.0, fixed);
    pitch_limit_ = declare_parameter("pitch_limit", 120.0, fixed);
    control_enabled_ = declare_parameter("control_enabled", true, fixed);
    fpv_pitch_sign_ = declare_parameter("fpv_pitch_command_sign", -1.0, fixed);
    if (fpv_pitch_sign_ != -1.0 && fpv_pitch_sign_ != 1.0) {
      throw std::invalid_argument("fpv_pitch_command_sign must be -1.0 or 1.0");
    }
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
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_states", rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
    auto command_qos = rclcpp::QoS(1).reliable().durability_volatile();
    command_qos.lifespan(rclcpp::Duration::from_seconds(command_timeout_));
    command_sub_ = create_subscription<msg::GimbalCommand>(
      "~/command", command_qos,
      [this](const msg::GimbalCommand & m) {manual(m);});
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

  void manual(const msg::GimbalCommand & m)
  {
    try {
      require_link();
      check_stamp(m.header, command_timeout_);
      if (last_feedback_.state == 1 || last_feedback_.state == 2 || last_feedback_.state == 3) {
        throw std::runtime_error("Device initializing, stopped or protected");
      }
      const std::array<uint8_t, 2> modes{m.roll_mode, m.pitch_mode};
      const std::array<uint8_t, 2> types{m.roll_type, m.pitch_type};
      const std::array<double, 2> values{m.roll, m.pitch};
      const std::array<double, 2> limits{roll_limit_, pitch_limit_};
      const std::array<bool, 2> center{m.center_roll, m.center_pitch};
      if (m.fpv_sensitivity < -16 || m.fpv_sensitivity > 15 ||
        ((m.roll_mode == 2) != (m.pitch_mode == 2)))
      {
        throw std::invalid_argument("Invalid sensitivity or mixed FPV modes");
      }
      auto candidate = control_;
      bool rate_active = false;
      for (std::size_t i = 0; i < 2; ++i) {
        if (types[i] == 1 && (age(camera_time_) > auxiliary_timeout_ ||
          control_.zoom == 0 || control_.vertical_fov == 0))
        {
          throw std::invalid_argument("Proportional rate requires fresh nonzero zoom/FOV");
        }
        if ((modes[i] != 1 && modes[i] != 2) || types[i] > 2 ||
          (modes[i] == 2 && types[i] != 0) || !std::isfinite(values[i]) ||
          std::abs(values[i]) > (types[i] == 0 ? limits[i] : max_rate_))
        {
          throw std::invalid_argument("Invalid axis mode, non-finite value or exceeded limit");
        }
        auto & a = candidate.axes[i];
        a.mode = modes[i];
        a.type = types[i];
        const double sign = i == 1 && modes[i] == 2 ? fpv_pitch_sign_ : 1.0;
        a.value = quantize(sign * values[i], types[i] == 0 ? 0.01 : 0.1);
        if (center[i]) {
          a.center = !a.center;
        }
        rate_active = rate_active || types[i] != 0;
      }
      // Horizontal C20D has no independently commanded yaw motor.
      candidate.axes[2].mode = m.roll_mode == 2 ? 2 : 1;
      candidate.axes[2].type = m.roll_mode == 2 ? 0 : 2;
      candidate.axes[2].value = 0;
      candidate.sensitivity = m.fpv_sensitivity;
      control_ = candidate;
      command_time_ = Clock::now();
      rate_active_ = rate_active;
      if (active_mode_ != 4) {
        queue_command(4);
        active_mode_ = 4;
      }
    } catch (const std::exception & e) {
      ++rejected_commands_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected command: %s", e.what());
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
    state_pub_->publish(m);
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
  double fpv_pitch_sign_ = -1.0;
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
  rclcpp::Subscription<msg::GimbalCommand>::SharedPtr command_sub_;
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
