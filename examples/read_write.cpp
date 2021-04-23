//
// Created by fs on 2021-04-23.
//

#include <poll.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include "serial/serial.h"

class SerialTerminal {
 public:
  SerialTerminal(std::string dev_path, int baudrate)
      : serial_dev_path_(std::move(dev_path)),
        serial_baudrate_(baudrate),
        serial_fd_(
            open_serial(serial_dev_path_.c_str(), serial_baudrate_, false)),
        need_exit_(false) {
    assert(serial_fd_ >= 0);
    serial_read_thread_ = std::thread([this]() {
      struct pollfd fds[] = {{.fd = serial_fd_, .events = POLLIN}};
      char buf[4 * 1024 + 1];
      while (!need_exit_) {
        poll(fds, sizeof(fds) / sizeof(fds[0]), -1);
        ssize_t num = read(serial_fd_, buf, sizeof(buf) - 1);
        if (num > 0) {
          buf[num] = '\0';
          printf("%s", buf);
        }
      }
    });
    serial_read_thread_.detach();

    serial_write_thread_ = std::thread([this]() {
      struct pollfd fds[] = {{.fd = STDIN_FILENO, .events = POLLIN}};
      char buf[4 * 1024 + 1];
      while (!need_exit_) {
        poll(fds, sizeof(fds) / sizeof(fds[0]), -1);
        ssize_t num = read(STDIN_FILENO, buf, sizeof(buf) - 1);
        if (num > 0) {
          write(serial_fd_, buf, num);
        }
      }
    });
    serial_write_thread_.detach();
  }

  ~SerialTerminal() {
    need_exit_ = true;
    serial_read_thread_.join();
    serial_write_thread_.join();
    close(serial_fd_);
  }

 private:
  std::string serial_dev_path_;
  int serial_baudrate_;
  int serial_fd_{-1};
  std::thread serial_read_thread_{};
  std::thread serial_write_thread_{};
  std::atomic_bool need_exit_;
};

int main() {
  new SerialTerminal("/dev/ttyUSB0", 115200);
  pthread_exit(nullptr);
}
