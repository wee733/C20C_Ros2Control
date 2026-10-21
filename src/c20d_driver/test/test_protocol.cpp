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
#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "c20d_driver/protocol.hpp"

using c20d_driver::Control;
using c20d_driver::Feedback;
using c20d_driver::Parser;
using c20d_driver::decode;
using c20d_driver::encode;

const std::array<uint8_t, 26> kOfficialReply = {
  0xb5, 0x9a, 0x24, 0x00, 0x19, 0x21, 0xff, 0xff, 0x00, 0x00, 0xfe, 0xff,
  0x09, 0x00, 0x8c, 0x35, 0xb9, 0x01, 0x52, 0xc9, 0x5f, 0x00, 0xfe, 0xff, 0x52, 0xcd};

TEST(Protocol, OfficialCommandExampleOne)
{
  Control c;
  c.command = 4;
  c.axes[0].type = 0;
  c.axes[2].mode = 0;
  const std::array<uint8_t, 40> expected = {
    0xa9, 0x5b, 0x20, 0x00, 0x10, 0, 0, 0x90, 0, 0, 0x80, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x61, 0xba};
  EXPECT_EQ(encode(c), expected);
}

TEST(Protocol, OfficialCommandExampleTwo)
{
  Control c;
  c.command = 4;
  c.axes[1].value = 10;
  c.carrier_valid = true;
  c.carrier_angle.fill(500);
  c.carrier_accel.fill(10);
  const std::array<uint8_t, 40> expected = {
    0xa9, 0x5b, 0x20, 0, 0x90, 0, 0, 0x90, 0x0a, 0, 0x90, 0, 0, 0x80,
    0xf4, 1, 0xf4, 1, 0xf4, 1, 10, 0, 10, 0, 10, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x98, 0xdb};
  EXPECT_EQ(encode(c), expected);
}

TEST(Protocol, OfficialReplyAndBitfieldErratum)
{
  Feedback f;
  ASSERT_TRUE(decode(kOfficialReply.data(), kOfficialReply.size(), f));
  EXPECT_EQ(f.firmware, 36);
  EXPECT_EQ(f.hardware_error, 0);
  EXPECT_TRUE(f.upright);  // 0x19 bit 0 is 1; the PDF example's text is inconsistent.
  EXPECT_EQ(f.state, 4);
  EXPECT_TRUE(f.temperature_ready);
  EXPECT_EQ(f.command, 4);
  EXPECT_EQ(f.command_status, 1);
  EXPECT_EQ(f.rate, (std::array<int16_t, 3>{-1, 0, -2}));
  EXPECT_EQ(f.attitude, (std::array<int16_t, 3>{9, 13708, 441}));
  EXPECT_EQ(f.relative, (std::array<int16_t, 3>{-13998, 95, -2}));
}

TEST(Protocol, StreamResynchronization)
{
  for (std::size_t split = 0; split <= kOfficialReply.size(); ++split) {
    Parser parser;
    auto a = parser.feed(kOfficialReply.data(), split);
    auto b = parser.feed(kOfficialReply.data() + split, kOfficialReply.size() - split);
    EXPECT_EQ(a.size() + b.size(), 1u);
  }
  Parser parser;
  std::vector<uint8_t> stream{0, 0xb5, 0xb5, 0x9a, 0x55};
  auto bad = kOfficialReply;
  bad[15] ^= 1;
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), kOfficialReply.begin(), kOfficialReply.end());
  stream.insert(stream.end(), kOfficialReply.begin(), kOfficialReply.end());
  EXPECT_EQ(parser.feed(stream.data(), stream.size()).size(), 2u);
  EXPECT_GE(parser.crc_errors(), 1u);
}

TEST(Protocol, EncodingSignedFieldsCameraAndValidation)
{
  Control c;
  c.command = 6;
  c.trigger = 7;
  c.sensitivity = -16;
  c.axes[0] = {true, 2, 0, -1234};
  c.vertical_fov = 60;
  c.zoom = 1000;
  c.target_angle = {1.0f, -2.0f};
  const auto p = encode(c);
  EXPECT_EQ(p[2], 0x37);
  EXPECT_EQ(p[3], 0x80);
  EXPECT_EQ(p[4], 0x28);
  EXPECT_EQ(p[5], 0x2e);
  EXPECT_EQ(p[6], 0xfb);
  EXPECT_EQ(p[26], 0x3c);
  EXPECT_EQ(p[27], 0xf4);
  EXPECT_EQ(p[28], 0x01);
  EXPECT_EQ(p[32], 0x80);
  EXPECT_EQ(p[33], 0x3f);
  EXPECT_EQ(p[37], 0xc0);
  c.axes[0].type = 2;
  EXPECT_THROW(encode(c), std::invalid_argument);
  EXPECT_THROW(c20d_driver::quantize(1e100, 0.01), std::out_of_range);
  EXPECT_THROW(
    c20d_driver::quantize(std::numeric_limits<double>::quiet_NaN(), 1.0),
    std::invalid_argument);
  const std::string check = "123456789";
  EXPECT_EQ(
    c20d_driver::crc16(reinterpret_cast<const uint8_t *>(check.data()), check.size()), 0x31c3);
}
