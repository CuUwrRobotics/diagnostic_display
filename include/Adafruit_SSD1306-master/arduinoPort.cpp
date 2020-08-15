/* This file is used to port and Arduino or arm-specific functions to
 * Raspberry Pi-compatible C++ to ease porting of libraries to the Pi.
 * 
 */
 
#include "arduinoPort.h"

// ####################################################
// I2C

// Merged with beginTransmission
//Begin writing to the wire buffer.
// void i2cBeginBuffer() {
// 	i2cBufferCount = 0;
// 	memset(i2c_buffer, 0, I2C_BUFFER_LENGTH);
// }

// WireWire(){}

uint8_t Wirewrite(uint8_t data) {
	if(i2cBufferCount < I2C_BUFFER_LENGTH){
		i2c_buffer[i2cBufferCount] = data;
		i2cBufferCount++;
	}
	else // Buffer is full; do nothing
        return 0;
	
    //ROS_INFO("%d", data);
    return 1;
}

uint8_t WireendTransmission() {
    if(i2cBufferCount == 0)
        return 0;
    else if (RASPI_I2C_MAX_BUFFER < i2cBufferCount)
        return 1;
	static int s;
    s = write(i2cDeviceFile, i2c_buffer, i2cBufferCount);
    if(s != i2cBufferCount){
        return 4;
	}
    i2cBufferCount = 0;
    return 0; // Success
    // Normally, other returns exist to flag ACK errros. Not sure how to check those on raspi.
}

int WirebeginTransmission() {
    // Prepare the buffer for data
	i2cBufferCount = 0;
	memset(i2c_buffer, 0, I2C_BUFFER_LENGTH);

    return 1;
}


int Wirebegin(int addr) {
    // Gets integer file descriptor number.
    // Note to self: DO NOT put this in the if statement! IT WILL FAIL!
    i2cDeviceFile = open(I2C_DEVICE_FILE, O_RDWR);

    // Error occurred.
    if(i2cDeviceFile < 0)
        ROS_ERROR("Could not get file descriptor. Errno: %s", std::strerror(errno));
    ROS_DEBUG("STARTING ON 0d%d", i2caddr);
    ROS_DEBUG("I2C interface file #%d", i2cDeviceFile);
    //i2cDeviceFile = 3;
    int s;
    s = ioctl(i2cDeviceFile, I2C_SLAVE, addr);
    if(s == -1){
        ROS_ERROR("Value returned: %d", s);
        ROS_ERROR("errno: %s", std::strerror(errno));
        ROS_ERROR("File: %d", i2cDeviceFile);
		return 0;
    }
    return 1;
}