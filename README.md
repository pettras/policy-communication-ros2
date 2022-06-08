# policy-communication-ros2

## Run

#### Terminal 1
ros2 run master policyNode 

#### Terminal 2
sudo chmod 777 /dev/ttyUSB1 

ros2 run master gripperNode 

## Requirements

A Zivid node running with the following code: https://github.com/pettras/zivid-ros2

A ROS2 interface with a KUKA iiwa 14: https://github.com/tingelst/sunrisedds
