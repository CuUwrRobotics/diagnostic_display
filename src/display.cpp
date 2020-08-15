#include "ros/ros.h"
#include "std_msgs/String.h"

#include "display.h"

// This defines all of the variables to check against when error checking.
#include "variables_to_check.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define WIDTH 128 // OLED display width, in pixels
#define HEIGHT 64 // OLED display height, in pixels

#define delay(a) usleep(a*1000)

// Address is bitshifted to remove the R/W bit.
#define SSD_ADDR (0x78 >> 1) // Equivilent to 3C.



// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

void childProcess(int errorPipe[], int dataPipe[], int argc, char** argv) ;
void parentProcess(int errorPipe[], int dataPipe[]);

// persistent errors: these errors trip once and remain tripped until 
// the program is restarted, even if the error goes away. This is used
// because the display will be out of sight underwater, and CPU/RAM/other
// errors are temporary but very important
bool ram_error = false, cpu_error = false, amperage_error = false, 
                  voltage_error = false, cpu_temp_error = false;


string GetStdoutFromCommand(string cmd) {

  string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
      pclose(stream);
  }
  return data;
}

string combineStrings(string s1, string s2, string s3 = "", string s4 = "", string s5 = "", string s6 = ""){
  s1 += s2;
  s1 += s3;
  s1 += s4;
  s1 += s5;
  s1 += s6;
  return s1;
}

long long int getIntFromString(string str){
  string numbers = "";
  long long int toReturn = 0;
  for(int i = 0; i < str.length(); i++)
    if(str.at(i) - '0' >= 0 && str.at(i) - '0' <= 9) 
      numbers += str.at(i);
    else if (str.at(i) == '.') break; // stop at decimals
  for(int i = 0; i < numbers.length(); i++){
    toReturn = toReturn * 10 + numbers.at(i) - '0';
  }
  return toReturn;
}

 static unsigned char toCidr(int* ipbytes) {
      unsigned char netmask_cidr;
      netmask_cidr=0;
      for (int i=0; i<4; i++) {
          switch(ipbytes[i]) {
              case 0x80: netmask_cidr+=1; break;
              case 0xC0: netmask_cidr+=2; break;
              case 0xE0: netmask_cidr+=3; break;
              case 0xF0: netmask_cidr+=4; break;
              case 0xF8: netmask_cidr+=5; break;
              case 0xFC: netmask_cidr+=6; break;
              case 0xFE: netmask_cidr+=7; break;
              case 0xFF: netmask_cidr+=8; break;
              
              default: return netmask_cidr; break;
          }
      }
      return netmask_cidr;
  }


static const char hexDigits[] = "0123456789ABCDEF";


int main(int argc, char **argv)
{
  //long long int begin = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch();
  //wiringPiSetup();
  //pinMode(8,OUTPUT);
  //digitalWrite(8,LOW);
    
  int errorPipe[2]; // Error bytes pipe variable
  int dataPipe[2]; // Diagnostics data (eg. temp, IP address) pipe variable
  
  if(pipe(errorPipe) < 0) // Start pipe
    ROS_ERROR("The ERROR pipe creation failed. errno: %s", std::strerror(errno));
  if(pipe(dataPipe) < 0) // Start pipe
    ROS_ERROR("The DATA pipe creation failed. errno: %s", std::strerror(errno));
    
  if (fcntl(errorPipe[0], F_SETFL, O_NONBLOCK) < 0)
    ROS_ERROR("fcntl for the error pipe has failed. errno: %s", std::strerror(errno));
    
      if (fcntl(dataPipe[0], F_SETFL, O_NONBLOCK) < 0)
    ROS_ERROR("fcntl for the data pipe has failed. errno: %s", std::strerror(errno));
    
  // Fork processes.
  int forkReturn = fork();
  switch(forkReturn) {
    
    //error
    case -1:
      ROS_ERROR("Forking failed. errno: %s", std::strerror(errno));
      break;
    
    // Child process
    case 0:
      childProcess(errorPipe, dataPipe, argc, argv);
      break;
      
    // Parent process
    default:
      ROS_INFO("Child process PID: %d", forkReturn);
      parentProcess(errorPipe, dataPipe);
      break;
  }

  return 0;
}

void childProcess(int errorPipe[], int dataPipe[], int argc, char** argv) { 
  ROS_INFO("Child process now running.");
  
  close(errorPipe[0]);
  close(dataPipe[0]);
  // Stroes all erros that have occured.
  static unsigned char errorBytes[PIPE_BUFFER_SIZE]; 
  string str; //Placeholder string
  bool updatedThisSecond = false, networkWorking = true, rosInitialized = false;  
  
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;
  typedef std::chrono::duration<float> fsec;
  auto t0 = Time::now();
  static char diagnosticData[DATA_PIPE_SIZE];
  
  while(true){
    // Get the time
    auto t1 = Time::now();
    fsec fs = t1 - t0;
    ms d = std::chrono::duration_cast<ms>(fs);
    
    if(fmod(TIME_SEC, SECONDS_DELAY) == 0 && !updatedThisSecond) { // Every few seconds perform updates
      updatedThisSecond = true;
      
      memset(errorBytes, 0b11111111, PIPE_BUFFER_SIZE); // Start the array with 'all good'
      //memset(errorBytes, 0x5D, PIPE_BUFFER_SIZE);
      string strCommand; // String to create commands in
      networkWorking = true;
      
      
      // Networking checks
      // Check if interface is up (other checks won't work if it isn't.)
      // cat /sys/class/net/NETWORK_INTERFACE/carrier
      strCommand = "cat /sys/class/net/";
      strCommand += NETWORK_INTERFACE;
      strCommand += "/carrier";
      str = GetStdoutFromCommand(strCommand);
      if(str.at(0) != '1') { // NETWORK_INTERFACE is not up
        errorBytes[0] &= 0b01111111; // LSB FIRST
        networkWorking = false;
      }
      
      if(networkWorking){
        // Check if the control device is available
        //traceroute -4 -w 1 -q 1 CONTROL_DEVICE_IP_ADDR | grep 'CONTROL_DEVICE_IP_ADDR'
        strCommand = "ping ";
        strCommand += CONTROL_DEVICE_IP_ADDR;
        strCommand += " -c 1 -w 1 | grep '64 bytes from ";
        strCommand += CONTROL_DEVICE_IP_ADDR;
        strCommand += "'";
        str = GetStdoutFromCommand(strCommand);
        if(str.length() < 1) // Check for number of character
          errorBytes[0] &= 0b11011111; // LSB FIRST
      
        // Check for hostname of control device
        //traceroute -4 -w 1 -q 1 CONTROL_DEVICE_HOSTNAME | grep 'CONTROL_DEVICE_HOSTNAME'
        strCommand = "ping ";
        strCommand += CONTROL_DEVICE_HOSTNAME;
        strCommand += " -c 1 -w 1 | grep '64 bytes from ";
        strCommand += CONTROL_DEVICE_HOSTNAME;
        strCommand += "'";
        str = GetStdoutFromCommand(strCommand);
        if(str.length() < 1) // Check for number of character
          errorBytes[0] &= 0b11011111; // LSB FIRST
      } else errorBytes[0] &= 0b11011111;
      
      unsigned char ipAddr[5], correctIpAddr[4];
      int subnet[4];
      // Check that IP/subnet is correct on NETWORK_INTERFACE
      if(networkWorking){
        strCommand = "ifconfig ";
        strCommand += NETWORK_INTERFACE;
        strCommand += " | grep inet";
        str = GetStdoutFromCommand(strCommand);
        sscanf(str.c_str(), "        inet %u.%u.%u.%u   netmask %d.%d.%d.%d", &ipAddr[0], 
                &ipAddr[1], &ipAddr[2], &ipAddr[3], &subnet[0], &subnet[1], &subnet[2], &subnet[3]);
        // Write diagnostics data
        for(int i = 0; i < 4; i++)
          diagnosticData[IP_ADDR_PT1_DATA + i] = ipAddr[i];
        diagnosticData[SUBNET_CIDR_DATA] = toCidr(subnet);
        // Get correct IP address and parse into ints.
        string correctIP = ROBOT_IP;
        sscanf(correctIP.c_str(), "%u.%u.%u.%u", &correctIpAddr[0], &correctIpAddr[1], 
                &correctIpAddr[2], &correctIpAddr[3]);
        bool ipGood = true;
        for(int i = 0; i < 4; i++)
          if(diagnosticData[IP_ADDR_PT1_DATA + i] != correctIpAddr[i]) // Check IP
            ipGood = false;
        if(diagnosticData[SUBNET_CIDR_DATA] != ROBOT_CIDR_SUBNET) // Check subnet
          ipGood = false;
        if(!ipGood)
          errorBytes[0] &= 0b10111111; // LSB FIRST
      } else {
        errorBytes[0] &= 0b10111111; // LSB FIRST
        for(int i = 0; i < 5; i++)
          diagnosticData[IP_ADDR_PT1_DATA + i] = 0; // no network => no ip
      }
      
      // ROS digit
      // rostopic list | grep /rosout
        //str = GetStdoutFromCommand("rostopic list | grep /rosout");
      bool rosRunningLocally = ROS_RUNNING, rosOnNetwork = ROS_RUNNING_ON_CONTROLLER;
      if(!(rosRunningLocally || rosOnNetwork)){ // either one running, not both
        errorBytes[1] &= 0b01111111;
        diagnosticData[ROS_MASTER_LOCAL] = 0;
      }
      else if (rosRunningLocally && !rosOnNetwork) // Is ROS is active locally, on the network, or at all? 
        diagnosticData[ROS_MASTER_LOCAL] = 1;
      else if (rosOnNetwork && !rosRunningLocally)
        diagnosticData[ROS_MASTER_LOCAL] = 2;
      else if (rosOnNetwork && rosRunningLocally){ // Both can't run
        errorBytes[1] &= 0b01111111;
        diagnosticData[ROS_MASTER_LOCAL] = 3;
      }
      
        
        
      // System digit
      // Check ram total
      strCommand = "cat /proc/meminfo | grep -i '";
      strCommand += RAM_TOTAL;
      strCommand += "'";
      str = GetStdoutFromCommand(strCommand);
      double totalRam = getIntFromString(str); 
      
      // Check ram free
      strCommand = "cat /proc/meminfo | grep -i '";
      strCommand += RAM_FREE;
      strCommand += "'";
      str = GetStdoutFromCommand(strCommand);
      double freeRam = getIntFromString(str); 
      
      // Setup data to send to display. Conversion assumes KiB, not KB.
      diagnosticData[USED_RAM_GB_DATA] = ((totalRam - freeRam) / totalRam) * 100;
      
      // Check swap total
      strCommand = "cat /proc/meminfo | grep -i '";
      strCommand += SWAP_TOTAL;
      strCommand += "'";
      str = GetStdoutFromCommand(strCommand);
      double totalSwap = getIntFromString(str); 
      
      // Check swap free
      strCommand = "cat /proc/meminfo | grep -i '";
      strCommand += SWAP_FREE;
      strCommand += "'";
      str = GetStdoutFromCommand(strCommand);
      double freeSwap = getIntFromString(str); 
      
      // Setup data to send to display. Conversion assumes KiB, not KB.
      diagnosticData[USED_SWAP_GB_DATA] = (totalSwap - freeSwap) / totalSwap * 100;
      
      double freeMemPct = ((totalRam + totalSwap) - (freeSwap + freeRam))
                                / (totalRam + totalSwap);
      if(freeMemPct * 100 >= MAX_RAM_USAGE)
        ram_error = true;
      if(ram_error)
        errorBytes[3] &= 0b11011111;
      
      str = GetStdoutFromCommand("/opt/vc/bin/vcgencmd measure_temp");
      int cpuTemp = getIntFromString(str);
      if(cpuTemp >= MAX_CPU_TEMP)
        cpu_temp_error = true;
      if(cpu_temp_error)
        errorBytes[3] &= 0b01111111;
        
      diagnosticData[TEMP_DATA] = cpuTemp; // Setup data to send to display
      
      // Send diagnostic data to be shown on display
      write(dataPipe[1], diagnosticData, DATA_PIPE_SIZE);

    
      
      // Write the troubleshooting line based on the error states in errorBytes
      str = "";
      static char hexDigit;
      for(int i = 0; i < PIPE_BUFFER_SIZE; i++){
        hexDigit = hexDigits[(i%2)?((errorBytes[i / 2])&0xf):((errorBytes[i / 2]>>4))];
        str += hexDigit;
      }
      write(errorPipe[1], str.c_str(), PIPE_BUFFER_SIZE);
      
    // Prevents multiple updates per second:
    } else if (TIME_MILLIS % SECONDS_DELAY != 0) updatedThisSecond = false;
    delay(1000);
  }
  return;
}

void parentProcess(int errorPipe[], int dataPipe[]) {
  ROS_INFO("Parent process running");
  close(errorPipe[1]);
  close(dataPipe[1]);
  
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SSD_ADDR)) { // Address 0x3C for 128x64
    ROS_ERROR("SSD1306 initialization failed");
    
  }
  
  display.ssd1306_command(0xa5); // Display ALL ON (test)
  delay(500);
  display.ssd1306_command(0xa4); // Display normal
  
  display.clearDisplay();
  display.display();
  
  display.drawBitmap(0, 0, logo, logo_w, logo_h, 1);
  display.write(0, 56, "OS: " + GetStdoutFromCommand("uname -r"));
  display.write(0, 48, "ROS: " + GetStdoutFromCommand("echo $ROS_DISTRO") + " v" + GetStdoutFromCommand("echo $ROS_VERSION"));
  display.write(88, 10, DIAGNOSTICS_VERSION, true, true);
  ROS_INFO("Showing splash screen for 5 seconds.");
  display.display();
  delay(5000);


  
  
  display.clearDisplay();
  ROS_DEBUG("Clearing screen; starting program.");
  display.display();
  
  
  // Stores all errors
  static unsigned char hexBytes[PIPE_BUFFER_SIZE]; 
  static unsigned char prevHexBytes[PIPE_BUFFER_SIZE]; 
  char errorPipeBuffer[PIPE_BUFFER_SIZE], dataPipeBuffer[DATA_PIPE_SIZE], oldDataPipe[DATA_PIPE_SIZE];
  memset(hexBytes, 0, PIPE_BUFFER_SIZE);
  memset(prevHexBytes, 0, PIPE_BUFFER_SIZE);
  memset(errorPipeBuffer, 0, PIPE_BUFFER_SIZE);
  
  // Data to be recieved from child process and displayed.
  // ipAddr array includes the CIDR notation at the end (or it would be [4])
  static unsigned char temp, usedSwap, usedRam, ipAddr[5], rosHost;
  
  static int pipeReadReturn, pipeReadReturn1;
  bool flashColor = false, heartbeat = false, dataUpdated[DATA_PIPE_SIZE];
  string str; // Placeholder string
  
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;
  typedef std::chrono::duration<float> fsec;
  auto t0 = Time::now();
  long previousFlashTime;
  
  while(true) {
    auto t1 = Time::now();
    fsec fs = t1 - t0;
    ms d = std::chrono::duration_cast<ms>(fs);
    
    // Invert display colors once in a while to avoid pixel burn-in
    display.invertDisplay(TIME_SEC % OLED_COLOR_INVERT_PERIOD * 2 < OLED_COLOR_INVERT_PERIOD);
    
    display.clearDisplay(); // Reset display
    
    // Sets color of heartbeat pixel which indicates that the code is running
    
    //Fill troubleshooting line based on any errors
    memset(hexBytes, 0b00000000, PIPE_BUFFER_SIZE); // Start the array with 'all bad'
    
    pipeReadReturn = read(errorPipe[0], errorPipeBuffer, PIPE_BUFFER_SIZE);
    if(pipeReadReturn == -1) {
      if(errno == EAGAIN) {
        // Pipe was empty - use previous data.
      memcpy(hexBytes, prevHexBytes, PIPE_BUFFER_SIZE);
      }
    }
    else if (pipeReadReturn == 0) {
      // Child process stopped running
      ROS_INFO("Child stopped.");
      close(errorPipe[0]);
      return;
    }
    else {
      // Data sent in pipe; get it into errorBytes.
      memcpy(hexBytes, errorPipeBuffer, PIPE_BUFFER_SIZE);
      heartbeat = true;
      ROS_DEBUG("Data recieved: %s", hexBytes);
    }
    
    pipeReadReturn1 = read(dataPipe[0], dataPipeBuffer, 32);
    if(pipeReadReturn1 == -1) {
      if(errno == EAGAIN) {
        // Pipe was empty - use previous data.
      }
    }
    else if (pipeReadReturn1 == 0) {
      // Child process stopped running
      ROS_INFO("Child stopped.");
      close(dataPipe[0]);
      return;
    }
    else {
      // Data sent in pipe; get it into errorBytes.
      temp = dataPipeBuffer[TEMP_DATA];
      usedRam = dataPipeBuffer[USED_RAM_GB_DATA];
      usedSwap = dataPipeBuffer[USED_SWAP_GB_DATA];
      ipAddr[0] = dataPipeBuffer[IP_ADDR_PT1_DATA];
      ipAddr[1] = dataPipeBuffer[IP_ADDR_PT2_DATA];
      ipAddr[2] = dataPipeBuffer[IP_ADDR_PT3_DATA];
      ipAddr[3] = dataPipeBuffer[IP_ADDR_PT4_DATA];
      ipAddr[4] = dataPipeBuffer[SUBNET_CIDR_DATA];
      rosHost = dataPipeBuffer[ROS_MASTER_LOCAL];
      
      
      for(int i = 0; i < DATA_PIPE_SIZE; i++)
        if(oldDataPipe[i] != dataPipeBuffer[i])
          dataUpdated[i] = true;
      memcpy(oldDataPipe, dataPipeBuffer, DATA_PIPE_SIZE);
    }
    
    // Get the time again
    t1 = Time::now();
    fs = t1 - t0;
    d = std::chrono::duration_cast<ms>(fs);
    
    // Write data to screen
    if(truncl(TIME_MILLIS / FLASH_TIME_MS) != previousFlashTime) {
      previousFlashTime = truncl(TIME_MILLIS / FLASH_TIME_MS);
      
      //str = "ROS: ";
      str = (rosHost == 2) ? combineStrings("Host: ", CONTROL_DEVICE_HOSTNAME) : 
                            ((rosHost == 1) ? combineStrings("Host: ", ROBOT_HOSTNAME) : 
                            ((rosHost == 0) ? "NO ROSCORE HOST!" : "MULTIPLE ROS HOSTS!"));
      display.write(0, 9, str, dataUpdated[ROS_MASTER_LOCAL]);

      // Write diagnostic data to screen.
      str = "IP: ";
      str += to_string(ipAddr[0]).c_str();
      str += ".";
      str += to_string(ipAddr[1]).c_str();
      str += ".";
      str += to_string(ipAddr[2]).c_str();
      str += ".";
      str += to_string(ipAddr[3]).c_str();
      str += "/";
      str += to_string(ipAddr[4]).c_str();
      str += "";
      display.write(0, 1, str, dataUpdated[IP_ADDR_PT1_DATA] || dataUpdated[IP_ADDR_PT2_DATA] || 
                          dataUpdated[IP_ADDR_PT3_DATA] || dataUpdated[IP_ADDR_PT4_DATA] || 
                          dataUpdated[SUBNET_CIDR_DATA]);
      
      str = "CPU: ";
      str += to_string(temp).c_str();
      str += 248;
      str += "C";
      display.write(0, 17, str, dataUpdated[TEMP_DATA], false);
      
      // Write diagnostic data to screen.
      display.write(73, 17, "RAM:", false, false);
      str = to_string(usedRam).c_str();
      str += "%";
      display.write(0, 17, str, dataUpdated[USED_RAM_GB_DATA], true);
      
      
      // Write diagnostic data to screen.
      display.write(73, 25, "SWAP:", false, false);
      str = to_string(usedSwap).c_str();
      str += "%";
      display.write(0, 25, str, dataUpdated[USED_SWAP_GB_DATA], true);
      
      // Adds a better background on error text for readability.
      display.fillRect(0, 55, 128, 9, 1);

      string str1 = "";
      for(int i = 0; i < PIPE_BUFFER_SIZE; i++) str1 += hexBytes[i];
      
      // Flash error digits
      if(flashColor)
        display.writeError(1, 56, str1, 'F', true); 
      else 
        display.write(1, 56, str1, true); 
        
      //Draws a 'heartbeat' on the top of the screen to indicate that
      // the code is still functioning by changing colors on update.
      if(heartbeat)
        display.drawFastHLineInternal(0, 0, 128, 1); // Blinks on every update.
        //display.drawPixel(0, 0, 1); // Blinks on every update.
      display.display();
      
      heartbeat = false;
      flashColor = !flashColor;
      for(int i = 0; i < DATA_PIPE_SIZE; i++)
        dataUpdated[i] = false;
    }
    memcpy(prevHexBytes, hexBytes, PIPE_BUFFER_SIZE);
  }
}

