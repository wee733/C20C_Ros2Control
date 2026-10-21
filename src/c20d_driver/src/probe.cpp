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
#include <exception>
#include <iomanip>
#include <iostream>
#include <thread>

#include "c20d_driver/protocol.hpp"
#include "c20d_driver/serial_port.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: c20d_probe /dev/ttyXXX (query command 0 only, 2 seconds)\n";
    return 2;
  }
  try {
    c20d_driver::SerialPort port;
    port.open(argv[1]);
    c20d_driver::Parser parser;
    const auto query = c20d_driver::encode(c20d_driver::Control{});
    std::size_t frames = 0;
    std::array<uint8_t, 1024> buffer{};
    for (int i = 0; i < 200; ++i) {
      port.write(query.data(), query.size());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      for (const auto & f : parser.feed(buffer.data(), port.read(buffer.data(), buffer.size()))) {
        if (frames++ == 0) {
          std::cout << "VALID C20D RESPONSE " << argv[1] << " firmware_raw=" << +f.firmware <<
            " hardware_error=" << +f.hardware_error << " state=" << +f.state <<
            " temperature_ready=" << f.temperature_ready << " upright=" << f.upright << '\n';
          std::cout << "relative_deg pitch=" << f.relative[0] * 0.01 <<
            " roll=" << f.relative[1] * 0.01 << " yaw=" << f.relative[2] * 0.01 << '\n';
          std::cout << "RAW ";
          for (auto byte : f.raw) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << +byte << ' ';
          }
          std::cout << std::dec << '\n';
        }
      }
    }
    std::cout << "frames=" << frames << " crc_errors=" << parser.crc_errors() <<
      " discarded_bytes=" << parser.discarded_bytes() << '\n';
    return frames ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
