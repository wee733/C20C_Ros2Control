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
#include <pty.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <string>
#include <system_error>
#include <thread>

#include "c20d_driver/serial_port.hpp"

TEST(Serial, PseudoTerminalRoundTripAndExclusiveOpen)
{
  int master = -1;
  int slave = -1;
  std::array<char, 128> name{};
  ASSERT_EQ(openpty(&master, &slave, name.data(), nullptr, nullptr), 0);
  c20d_driver::SerialPort port;
  port.open(name.data());
  c20d_driver::SerialPort second;
  EXPECT_THROW(second.open(name.data()), std::system_error);
  const std::array<uint8_t, 5> request{0, 0xa9, 0x5b, 0xff, 0x10};
  port.write(request.data(), request.size());
  std::array<uint8_t, 128> buffer{};
  ASSERT_EQ(::read(master, buffer.data(), buffer.size()), 5);
  EXPECT_EQ(buffer[1], 0xa9);
  ASSERT_EQ(::write(master, request.data(), 2), 2);
  std::size_t count = 0;
  for (int attempt = 0; attempt < 100 && count == 0; ++attempt) {
    count = port.read(buffer.data(), buffer.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(count, 2u);
  EXPECT_EQ(buffer[1], 0xa9);
  port.close();
  second.open(name.data());
  second.close();
  ::close(slave);
  ::close(master);
}
