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
#ifndef C20D_DRIVER__SERIAL_PORT_HPP_
#define C20D_DRIVER__SERIAL_PORT_HPP_

#include <termios.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace c20d_driver
{
/// Linux nonblocking 115200 8N1 UART; exclusive ownership, bounded writes, restored settings.
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  void open(const std::string & path);
  void close();
  bool is_open() const {return fd_ >= 0;}
  std::size_t read(uint8_t * data, std::size_t capacity);
  void write(const uint8_t * data, std::size_t size);

private:
  int fd_ = -1;
  termios original_{};
  bool restore_ = false;
  bool exclusive_ = false;
};
}  // namespace c20d_driver
#endif  // C20D_DRIVER__SERIAL_PORT_HPP_
