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
#ifndef C20D_DRIVER__PROTOCOL_HPP_
#define C20D_DRIVER__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace c20d_driver
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::size_t kTxSize = 40;
constexpr std::size_t kRxSize = 26;

struct AxisControl
{
  bool center = false;
  uint8_t mode = 1;
  uint8_t type = 2;
  int16_t value = 0;
};

struct Control
{
  uint8_t trigger = 0;
  uint8_t command = 0;
  int8_t sensitivity = 0;
  std::array<AxisControl, 3> axes{};  // Roll, pitch, yaw.
  bool carrier_valid = false;
  std::array<int16_t, 3> carrier_angle{};
  std::array<int16_t, 3> carrier_accel{};  // North, east, up.
  uint8_t vertical_fov = 0;
  uint32_t zoom = 0;
  std::array<float, 2> target_angle{};  // Degrees, right/down positive.
};

struct Feedback
{
  std::array<uint8_t, kRxSize> raw{};
  uint8_t firmware = 0;
  uint8_t hardware_error = 0;
  bool upright = false;
  uint8_t state = 0;
  bool temperature_ready = false;
  uint8_t command_status = 0;
  uint8_t command = 0;
  std::array<int16_t, 3> rate{};  // Wire order: pitch, roll, yaw; 0.1 deg/s.
  std::array<int16_t, 3> attitude{};  // Wire order: roll, pitch, yaw; 0.01 deg.
  std::array<int16_t, 3> relative{};  // Wire order: pitch, roll, yaw; 0.01 deg.
};

uint16_t crc16(const uint8_t * data, std::size_t size);
std::array<uint8_t, kTxSize> encode(const Control & control);
bool decode(const uint8_t * data, std::size_t size, Feedback & feedback);
int16_t quantize(double value, double scale);

/// Bounded stream parser: tolerates noise, partial reads, concatenation and bad CRCs.
class Parser
{
public:
  std::vector<Feedback> feed(const uint8_t * data, std::size_t size);
  void reset();
  uint64_t crc_errors() const {return crc_errors_;}
  uint64_t discarded_bytes() const {return discarded_bytes_;}

private:
  std::vector<uint8_t> buffer_;
  uint64_t crc_errors_ = 0;
  uint64_t discarded_bytes_ = 0;
};
}  // namespace c20d_driver
#endif  // C20D_DRIVER__PROTOCOL_HPP_
