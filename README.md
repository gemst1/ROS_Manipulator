# ROS_Manipulator

https://github.com/eYSIP-2017/eYSIP-2017_Robotic_Arm/wiki/Interfacing-MoveIt%21-with-Gazebo

- CAN communication

sudo slcand -o -c -f -s4 /dev/ttyUSB0 slcan0

sudo ifconfig slcan0 up

- CAN Communication monitor

candump slcan0


---------------------------------------------------------------------
- Install slcan module to Raspberry Pi </br>
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi


### CANUSB serial port latency set up
sudo apt-get install setserial</br>
setserial /dev/ttyUSBx low_latency
