# Robotnik EIP Scanner
A ROS1 package for interfacing with PLC's over EtherNet/IP protocol. Initial focus on the Schmersal PSC1-C-100 safety PLC.
- publish PLC state to a topic
- change PLC state using a service

## Inspiration

The goal is to reproduce the functionality of the package
- [safety_module](https://github.com/RobotnikAutomation/safety_module)

## Dependencies

### ROS dependencies

- robotnik_msgs
    robotnik_modbus_io: The communication with modbus id done through this node

### C++ dependencies
- [eip_scanner](https://github.com/nimbuscontrols/EIPScanner)

# Installation

Install [EIPScanner](https://github.com/nimbuscontrols/EIPScanner) library in /usr/local/include directory:
Change the CPP_VERSION from 20 to 14 in CMakeLists.txt for [ROS1 Melodic compatibility](http://wiki.ros.org/melodic/Migration)
```
set(CMAKE_CXX_STANDARD 14)
```

Compile EIPScanner
```
mkdir build
cd build
sudo cmake ..
sudo cmake --build . --target install
```

In order to run this node, and before that, in order to `catkin build` successfull, you must be able to find the library on runtime:
```
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

TODO: Wrap the eip_scanner library i

# Robot connection

In order to connect to the PLC, you must manually configure your *wired* connection. 
- In the wired settings, click Network -> Wired -> [gear] -> IPv4 -> Manual
- Add
```
address: 192.168.0.X (for example, replace X with 24)
netmask: 255.255.255.0
gateway: [blank]
```

- You can leave DNS and ROUTES as automatic

You can use the robot's ethernet port FB 2.2


## Temporary connection information
As shown in the launch file, the current IP address of the PLC being tested is 192.168.0.250 

# The first three robot switches
- OFF/ON
- AUTO/MA/MT
- LASER MUTE OFF/ON

- 1st switch: the robot is power off/on
- 2nd switch:
  - AUTO: (ignores the 3rd switch, powering OFF according to the laser input)
  - MA (manual): operate according to the 3rd switch
  - MT (maintenance):ignore the lasers
- 3rd switch: laser mute on/off

- MA + LASER MUTE OFF = AUTO
- MA + LASER MUTE ON = MT

# Explicit messaging: PSC1-C-100

## Reading

Byte 0, first 4 bits (decimal values):
(1-3 will be invisible, since the bus isn't initialized)
- 1 init
- 2 self check
- 3 initializing bus
- 4 running
- 5 stopped (probably can't get data either)
- 6 fatal error
- 7 alarm

Byte 0, last 4 bits:
live counter (probably not useful since it only can count to 8)

Byte 6, 7th bit
- 0 OK
- 1 error (note that if this bit = 1, then byte 0 should be a 6 or 7)

Byte5, bit 0: whether K3 and K4 are activated (inactive K3 stops the motors and inactive K4 puts on the breaks - K3 & K4 (in)activated together)
- 0 active (and thus the robot can move)
- 1 inactive (the robot can't move)

Byte5, bit 1: revision of K2
- 0 inactive
- 1 active

K2 gives power to the drivers, so if active, the motors have power. When there's an emergency stop, the revision of K2 will be active (1) for 30 ms after K3 and K4 are become inactive (0->1), and then the revision of K2 will be inactive (0) 

Byte 5, bit 2:
- 0 lasers are in error and/or there are objects in the protection zone
- 1 means lasers are not in error and no objects in protection zone

## Writing
- Can use Bytes 0 - 3
- Byte 0, bit 0: send 1 to switch on the buzzer

## When you turn on the robot, expect...
- Byte 0, first 4 bits should be a 4 (0b100)
- Byte 6, 7th bit: should be a 0
- Byte 5 bit 0: if the blue light is on, should be 0, but after you press it the blue light should turn off and this bit should become 1
