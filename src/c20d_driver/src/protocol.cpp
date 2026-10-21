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
#include "c20d_driver/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace c20d_driver
{
namespace
{
void put16(uint8_t * p, int16_t value)
{
  const auto bits = static_cast<uint16_t>(value);
  p[0] = static_cast<uint8_t>(bits);
  p[1] = static_cast<uint8_t>(bits >> 8);
}

int16_t get16(const uint8_t * p)
{
  const int value = p[0] | (static_cast<int>(p[1]) << 8);
  return static_cast<int16_t>(value >= 32768 ? value - 65536 : value);
}

void put32(uint8_t * p, uint32_t bits)
{
  for (unsigned i = 0; i < 4; ++i) {
    p[i] = static_cast<uint8_t>(bits >> (8 * i));
  }
}
}  // namespace

uint16_t crc16(const uint8_t * data, std::size_t size)
{
  // CRC-16/XMODEM: poly 0x1021, init 0, no reflection, xorout 0.
  uint16_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
  }
  return crc;
}

int16_t quantize(double value, double scale)
{
  if (!std::isfinite(value) || !std::isfinite(scale) || scale <= 0.0) {
    throw std::invalid_argument("Non-finite value or invalid scale");
  }
  const double raw = std::round(value / scale);
  if (!std::isfinite(raw) || raw < -32768 || raw > 32767) {
    throw std::out_of_range("Value exceeds signed 16-bit protocol range");
  }
  return static_cast<int16_t>(raw);
}

std::array<uint8_t, kTxSize> encode(const Control & c)
{
  if (c.command > 6 || c.trigger > 7 || c.sensitivity < -16 || c.sensitivity > 15 ||
    c.vertical_fov > 127 || c.zoom > 0xffffff)
  {
    throw std::invalid_argument("Invalid protocol control field");
  }
  std::array<uint8_t, kTxSize> p{};
  p[0] = 0xa9;
  p[1] = 0x5b;
  p[2] = static_cast<uint8_t>((c.command << 3) | c.trigger);
  p[3] = static_cast<uint8_t>((static_cast<uint8_t>(c.sensitivity) & 0x1f) << 3);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto & a = c.axes[i];
    if (a.mode > 2 || a.type > 2 || (a.mode == 2 && a.type != 0)) {
      throw std::invalid_argument("Invalid axis mode/type combination");
    }
    p[4 + 3 * i] = static_cast<uint8_t>((a.center ? 8 : 0) | (a.mode << 4) | (a.type << 6));
    put16(&p[5 + 3 * i], a.value);
    put16(&p[14 + 2 * i], c.carrier_angle[i]);
    put16(&p[20 + 2 * i], c.carrier_accel[i]);
  }
  p[13] = c.carrier_valid ? 0x80 : 0;
  put32(&p[26], c.vertical_fov | (c.zoom << 7));
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  for (std::size_t i = 0; i < 2; ++i) {
    if (!std::isfinite(c.target_angle[i])) {
      throw std::invalid_argument("Non-finite target angle");
    }
    uint32_t bits = 0;
    std::memcpy(&bits, &c.target_angle[i], sizeof(bits));
    put32(&p[30 + 4 * i], bits);
  }
  const auto crc = crc16(p.data(), kTxSize - 2);
  p[38] = static_cast<uint8_t>(crc >> 8);
  p[39] = static_cast<uint8_t>(crc);
  return p;
}

bool decode(const uint8_t * p, std::size_t size, Feedback & feedback)
{
  if (size != kRxSize || p[0] != 0xb5 || p[1] != 0x9a) {
    return false;
  }
  const auto crc = crc16(p, kRxSize - 2);
  if (p[24] != static_cast<uint8_t>(crc >> 8) || p[25] != static_cast<uint8_t>(crc)) {
    return false;
  }
  Feedback f;
  std::copy_n(p, kRxSize, f.raw.begin());
  f.firmware = p[2];
  f.hardware_error = p[3];
  f.upright = (p[4] & 1) != 0;
  f.state = (p[4] >> 1) & 7;
  f.temperature_ready = (p[4] & 0x10) != 0;
  f.command_status = p[5] & 7;
  f.command = p[5] >> 3;
  for (std::size_t i = 0; i < 3; ++i) {
    f.rate[i] = get16(&p[6 + 2 * i]);
    f.attitude[i] = get16(&p[12 + 2 * i]);
    f.relative[i] = get16(&p[18 + 2 * i]);
  }
  feedback = f;
  return true;
}

std::vector<Feedback> Parser::feed(const uint8_t * data, std::size_t size)
{
  std::vector<Feedback> result;
  for (std::size_t i = 0; i < size; ++i) {
    buffer_.push_back(data[i]);
    while (!buffer_.empty()) {
      if (buffer_[0] != 0xb5 || (buffer_.size() >= 2 && buffer_[1] != 0x9a)) {
        buffer_.erase(buffer_.begin());
        ++discarded_bytes_;
      } else if (buffer_.size() < kRxSize) {
        break;
      } else {
        Feedback feedback;
        if (decode(buffer_.data(), kRxSize, feedback)) {
          result.push_back(feedback);
          buffer_.clear();
        } else {
          ++crc_errors_;
          ++discarded_bytes_;
          buffer_.erase(buffer_.begin());
        }
      }
    }
  }
  return result;
}

void Parser::reset()
{
  buffer_.clear();
}
}  // namespace c20d_driver
