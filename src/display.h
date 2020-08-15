#include <iostream>
#include <chrono>
#include <ros/master.h>
#include <algorithm>
#include </usr/include/stdio.h>
#include </usr/include/fcntl.h>
#include </usr/include/unistd.h>

#include </home/pi/diagnostics_ws/src/diagnostic_display/include/Adafruit_SSD1306-master/SSD1306_GFX.cpp>
#include </home/pi/diagnostics_ws/src/diagnostic_display/include/Adafruit_SSD1306-master/SSD1306_GFX.h>
// #include </home/pi/diagnostics_ws/src/diagnostic_display/include/Adafruit_SSD1306-master/arduinoPort.h>

#include "arduinoPort.h"
//#include "arduinoPort.cpp"


using namespace std;
// Seconds to maintain display fore/background colors.
// Display will invert these colors to avoid burn-in. 
// 30 seconds will invert the display twice in a minute.
#define OLED_COLOR_INVERT_PERIOD 30

// This referrs to the version of the error codes at the bottom of the 
// screen. Max 6 characters.
#define DIAGNOSTICS_VERSION "v0.1"

// Size of the data sent through the pipe.
#define PIPE_BUFFER_SIZE 21

#define SECONDS_DELAY 3 // Seconds between running checks

// Time in milliseconds to flash problematic digits.
#define FLASH_TIME_MS 250

// Checks whther ROSCORE is running, locally or over the network.
#define ROS_RUNNING (GetStdoutFromCommand("rostopic list | grep /rosout").length() > 1)
#define ROS_RUNNING_ON_CONTROLLER (GetStdoutFromCommand(combineStrings("export ROS_MASTER_URI=http://", CONTROL_DEVICE_HOSTNAME, ":11311\nrostopic list | grep /rosout")).length() > 1)

// Time reading stuff
#define TIME_MILLIS (d.count())
#define TIME_SEC ((int)trunc(d.count() / 1000))

// The number of data values in the data pipe array
#define DATA_PIPE_SIZE 32
// The diagnostic data pipe sends an array with information at these addresses:
#define TEMP_DATA 0
#define USED_RAM_GB_DATA 1
#define USED_SWAP_GB_DATA 2
#define IP_ADDR_PT1_DATA 3
#define IP_ADDR_PT2_DATA 4
#define IP_ADDR_PT3_DATA 5
#define IP_ADDR_PT4_DATA 6
#define SUBNET_CIDR_DATA 7
#define ROS_MASTER_LOCAL 8
