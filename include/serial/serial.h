#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open a serial port
 * @param dev_path the path to the serial port e.g. /dev/ttyUSB0
 * @param speed baudrate, e.g. 115200
 * @param block block(1) or nonblock(0)
 * @return -1 Some errors have occurred, the errors will be printed to stderr
 * @return int File descriptor, similar to the return of @open()
 */
int open_serial(const char *dev_path, int speed, int block);

#ifdef __cplusplus
}
#endif
