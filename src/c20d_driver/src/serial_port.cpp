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
#include "c20d_driver/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <system_error>

namespace c20d_driver
{
SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & path)
{
  close();
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "open " + path);
  }
  try {
    if (flock(fd_, LOCK_EX | LOCK_NB) < 0 || ioctl(fd_, TIOCEXCL) < 0) {
      throw std::system_error(errno, std::generic_category(), "exclusive UART lock");
    }
    exclusive_ = true;
    if (tcgetattr(fd_, &original_) < 0) {
      throw std::system_error(errno, std::generic_category(), "tcgetattr");
    }
    restore_ = true;
    auto settings = original_;
    cfmakeraw(&settings);
    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &settings) < 0 || tcflush(fd_, TCIOFLUSH) < 0) {
      throw std::system_error(errno, std::generic_category(), "configure UART");
    }
  } catch (...) {
    close();
    throw;
  }
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    if (restore_) {
      tcsetattr(fd_, TCSANOW, &original_);
    }
    if (exclusive_) {
      ioctl(fd_, TIOCNXCL);
    }
    ::close(fd_);
    fd_ = -1;
    restore_ = false;
    exclusive_ = false;
  }
}

std::size_t SerialPort::read(uint8_t * data, std::size_t capacity)
{
  pollfd status{fd_, POLLIN, 0};
  if (poll(&status, 1, 0) < 0 && errno != EINTR) {
    throw std::system_error(errno, std::generic_category(), "poll UART");
  }
  if (status.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("UART disconnected");
  }
  const auto n = ::read(fd_, data, capacity);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    throw std::system_error(errno, std::generic_category(), "read UART");
  }
  return static_cast<std::size_t>(n);
}

void SerialPort::write(const uint8_t * data, std::size_t size)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  std::size_t offset = 0;
  while (offset < size) {
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("UART write timeout");
    }
    const auto n = ::write(fd_, data + offset, size - offset);
    if (n > 0) {
      offset += static_cast<std::size_t>(n);
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw std::system_error(errno, std::generic_category(), "write UART");
    } else {
      pollfd status{fd_, POLLOUT, 0};
      poll(&status, 1, 1);
    }
  }
}
}  // namespace c20d_driver
