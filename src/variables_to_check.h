/*Error bits: at the bottom of the display there are 21 hexadecimal characters.
  Each bit of each character represents a different possible error. Each
  hexadecimal digit should indicate a diffeent set of errors, like networking,
  ROS errors, power issues, etc
 When writing these bits, it is good practice to deticate certain characters to certain tasks.
  A hexadecimal digit has four different bits. The significance of the error
  should detemine how significant the issue is. For example, if 
  NETWORK_INTERFACE is DOWN, that means that the network cable is unplugged,
  which is more important than any other potential network issues.

 ERROR BIT DEFINITIONS (be descriptive! An error is no help if there's no clear explanation.)
 These are the human readable defenitions for error bits, with MSB first.
 If the following bits are LOW, the given error has occured.
 DIGIT 1: Networking
  MSB#1: NETWORK_INTERFACE not UP. Is a cable loose?
  BIT#2: My network address/hostname is not right. Check that ROBOT_IP 
            and ROBOT_HOSTNAME match system settings.
  BIT#3: Control device IP address OR hostname not responsing to traceroute pings sent 
            via bash command.
  LSB#4: 
 DIGIT 2: Networking 2 (reserved)
  MSB#1: 
  BIT#2: 
  BIT#3: 
  LSB#4: 
 DIGIT 3: roscore
  MSB#1: roscore is not running; ROS dependant scripts won't run.
  BIT#2: 
  BIT#3: 
  LSB#4: 
 DIGIT 4: System
  MSB#1: System temp rose above MAX_CPU_TEMP deg C at some point
  BIT#2: CPU usage rose above MAX_CPU_USAGE at some point (not implemented)
  BIT#3: RAM + swap usage rose above MAX_RAM_USAGE at some point.
  LSB#4: 
 DIGIT 5: Power
  MSB#1: Amperage on power line went above MAX_AMPERE at some point.
  BIT#2: Voltage on power line went below MIN_VOLTAGE at some point.
  BIT#3: One of the regulator's voltages went too low.
  LSB#4: One of the regulator's current went too high.

 TEMPLATE:
 DIGIT X:
  MSB#1: 
  BIT#2: 
  BIT#3: 
  LSB#4: 
  */

// IP address of the device on the surface which controlls the robot.
// This is statically defined on the device. 
#define CONTROL_DEVICE_IP_ADDR "192.168.1.1"

// Hostname of the surface control device. ROS uses this, not IP adresses.
// Note that this is defined locally, not by the actual hostname. 
// Defined in the file /etc/hosts.
#define CONTROL_DEVICE_HOSTNAME "raspi3" 

// The network interface is the physical connection that connects the device
// to the network. Ethernet on a Raspberry Pi 4 is 'eth0'. WiFI (for example, 
// wifi not used because water) is 'wlan0'.
#define NETWORK_INTERFACE "eth0"

// This device's IP address, then the CIDR notation of the subnet. See /etc/dhcpcd.conf
#define ROBOT_IP "192.168.1.2"
#define ROBOT_CIDR_SUBNET 30

// This device's hostname, as set in settings. Note that if this is wrong, 
// it may break nothing, but it will be very confusing to people working on 
// the device. This is set in the Pi's preferences.
#define ROBOT_HOSTNAME "raspi4"

// The topic that this script reads as a watchdog. This topic must have a 
// changed value from the last value every two seconds. 
#define CHATTER_TOPIC "chatter"

// Maximum CPU temp, deg C. It is recommended never to go above 80, so 70 
// should yield a good 'danger close' warning. 
#define MAX_CPU_TEMP 75

// Max CPU usage, pct
#define MAX_CPU_USAGE 90

// Max RAM usage, percent of total including swap
#define MAX_RAM_USAGE 90
// Values found with 'grep -i' to check ram in 'cat /proc/meminfo'
#define RAM_TOTAL "memtotal"
#define RAM_FREE "memfree"
#define SWAP_TOTAL "swaptotal"
#define SWAP_FREE "swapfree"

// Maximum amperage: Will trip an error if the measured power tether's
// amperage is ever recorded higher than this.
#define MAX_CURRENT 28

// Min voltage: If the tether's voltage is ever below this value, the error bit will trip.
#define MIN_VOLTAGE 45

// Regulators: enter a zero if a regulator should not be checked. The error
// bit will be tripped under the same conditions as above. 
// Change REGULATOR_COUNT to add more regulators.
// Regulator 0: 5V 3A TYP
// Regulator 1: 12V 5A TYP
#define REGULATOR_COUNT 2
double regulator_min_voltages[REGULATOR_COUNT] = {4.9, 11.5};
double regulator_max_current[REGULATOR_COUNT] = {2.7, 4.5};

