#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open a serial port
 * @param dev_path the path to the serial port e.g. /dev/ttyUSB0
 * @param speed baudrate, e.g. 115200
 * @param is_block block(1) or nonblock(0)
 * @return
 */
int serial_open(char *dev_path, int speed, int is_block);

#ifdef __cplusplus
}
#endif
