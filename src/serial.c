#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static inline int get_baudrate(int speed) {
  switch (speed) {
#ifdef B0
    case 0:
      return B0;
#endif
#ifdef B50
    case 50:
      return B50;
#endif
#ifdef B75
    case 75:
      return B75;
#endif
#ifdef B110
    case 110:
      return B110;
#endif
#ifdef B134
    case 134:
      return B134;
#endif
#ifdef B150
    case 150:
      return B150;
#endif
#ifdef B200
    case 200:
      return B200;
#endif
#ifdef B300
    case 300:
      return B300;
#endif
#ifdef B600
    case 600:
      return B600;
#endif
#ifdef B1200
    case 1200:
      return B1200;
#endif
#ifdef B1800
    case 1800:
      return B1800;
#endif
#ifdef B2400
    case 2400:
      return B2400;
#endif
#ifdef B4800
    case 4800:
      return B4800;
#endif
#ifdef B7200
    case 7200:
      return B7200;
#endif
#ifdef B9600
    case 9600:
      return B9600;
#endif
#ifdef B14400
    case 14400:
      return B14400;
#endif
#ifdef B19200
    case 19200:
      return B19200;
#endif
#ifdef B28800
    case 28800:
      return B28800;
#endif
#ifdef B57600
    case 57600:
      return B57600;
#endif
#ifdef B76800
    case 76800:
      return B76800;
#endif
#ifdef B38400
    case 38400:
      return B38400;
#endif
#ifdef B115200
    case 115200:
      return B115200;
#endif
#ifdef B128000
    case 128000:
      return B128000;
#endif
#ifdef B153600
    case 153600:
      return B153600;
#endif
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B256000
    case 256000:
      return B256000;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B500000
    case 500000:
      return B500000;
#endif
#ifdef B576000
    case 576000:
      return B576000;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B1152000
    case 1152000:
      return B1152000;
#endif
#ifdef B1500000
    case 1500000:
      return B1500000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
#ifdef B2500000
    case 2500000:
      return B2500000;
#endif
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
#ifdef B3500000
    case 3500000:
      return B3500000;
#endif
#ifdef B4000000
    case 4000000:
      return B4000000;
#endif
    default:
      return -1;
  }
}

// Set serial input/output baudrate
static int set_speed(int fd, int speed) {
  // flush serial
  tcflush(fd, TCIOFLUSH);

  // Can't find the corresponding baud rate
  int baud = get_baudrate(speed);
  if (baud < 0) {
    return -1;
  }

  struct termios opt;
  tcgetattr(fd, &opt);
  cfsetispeed(&opt, baud);
  cfsetospeed(&opt, baud);

  // Set baudrate
  if (tcsetattr(fd, TCSANOW, &opt) != 0) {
    perror("tcsetattr fd:");
    return -1;
  }

  return 0;
}

// Set parity
static int set_parity(int fd) {
  struct termios opt;
  tcgetattr(fd, &opt);

  // 8 data bits, parity is none
  opt.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  opt.c_oflag &= ~OPOST;
  opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  opt.c_cflag &= ~(CSIZE | PARENB);
  opt.c_cflag |= CS8;

  tcflush(fd, TCIFLUSH);

  if (tcsetattr(fd, TCSANOW, &opt) != 0) {
    perror("set attr parity error:");
    return -1;
  }

  return 0;
}

// Open a serial port
int serial_open(char *dev_path, int speed, int is_block) {
  int flag = O_RDWR;                      // read and write
  if (is_block == 0) flag |= O_NONBLOCK;  // block or nonblock

  int fd = open(dev_path, flag);
  if (fd < 0) {
    perror("Open device file err:");
    return -1;
  }

  if (set_speed(fd, speed) != 0) {
    close(fd);
    return -1;
  }

  if (set_parity(fd) != 0) {
    close(fd);
    return -1;
  }

  return fd;
}
