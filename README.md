# Author: Patrik Dominik Pördi
# ROS Humble C++ PubSub Example

## Overview

This ROS 2 package, has been created an autonomous movement by the turtlebot with obstacle avoidance

## Prerequisites

Before using this package, make sure you have the following dependencies:

- ROS 2 Humble (Installation guide: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html))
- [Colcon](https://colcon.readthedocs.io/en/released/)
- C++17 compatible compiler


Additionally, for simulation purposes, you will need to install the following ROS packages:

```bash
# Install Gazebo ROS packages for TurtleBot simulation
sudo apt -y install ros-humble-gazebo-ros-pkgs

# Install TurtleBot 3 packages
sudo apt -y install ros-humble-turtlebot3*
```

## Building and Installation

To build the package, follow these steps:

1. Clone the package into your ROS 2 workspace's `src/` directory:

```bash
git clone git@github.com:patrikpordi/ros2-turtlebot.git
```
----

To bag all the topics during launch use
```bash
ros2 ros2 launch ros2-turtlebot launch.py rosbag_record:=true
```
OR
Without bagging
```bash
ros2 launch ros2-turtlebot launch.py
```


Investigate the bag files:
```bash
ros2 bag info src/ros2-turtlebot/results/walking_bag/walking_bag_0.db3

ros2 bag play src/ros2-turtlebot/results/walking_bag/walking_bag_0.db3

```


## CppLint & CppCheck
   ```bash
   # Use the below command for cpp lint by moving to root directory of your workspace 
   cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint.txt

   # Use the below command for cpp check by moving to root directory of your workspace
   cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> results/cppcheck.txt
```



## Results

Can be found results/
   cppcheck.txt
   cpplint.txt
   walking_bag
