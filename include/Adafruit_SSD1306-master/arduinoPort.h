
#ifndef ARDUINOPORT_H
#define ARDUINOPORT_H


#include </usr/include/stdint.h>
#include </usr/include/linux/types.h>
#include </usr/include/linux/stat.h>
#include </usr/include/string.h>
#include </usr/include/stdlib.h>
#include </usr/include/stdio.h>
#include </usr/include/linux/ioctl.h>
#include </usr/include/linux/i2c.h>
#include </usr/include/linux/i2c-dev.h>
#include </usr/include/errno.h>

#include </usr/include/arm-linux-gnueabihf/sys/ioctl.h>
#include </usr/include/fcntl.h>
#include </usr/include/unistd.h>
#include </usr/include/linux/ioctl.h>
#include </usr/include/linux/i2c.h>
#include </usr/include/linux/i2c-dev.h>

#include "ros/ros.h"

using namespace std;


// ####################################################
// Single-line changes

// Arduino delay function is converted to linux sleep function in microseconds.
#define delay(a) usleep(a*1000) 


// ####################################################
// I2C

// setup variables
// Max bytes to buffer before sending. Default for arduino = 32
#define I2C_BUFFER_LENGTH 128 
#define RASPI_I2C_MAX_BUFFER 8192 // I think this is right.
#define I2C_DEVICE_FILE "/dev/i2c-1" // The actual file to write to for I2C in /dev/
uint8_t i2c_buffer[I2C_BUFFER_LENGTH]; // Byte buffer
uint8_t i2cBufferCount = 0, i2caddr;
int i2cDeviceFile;

// class Wire { 
    // public:
        uint8_t Wirewrite(uint8_t data);
        uint8_t WireendTransmission();
        int WirebeginTransmission();
        int Wirebegin(int addr);
// };

#endif