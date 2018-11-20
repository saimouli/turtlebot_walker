# Turtlebot Walker
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Project Overview
This project implements a roomba type behavior robot using turtlebot platform in ROS. This repository contains the following files: 
- include/turtlebot_walker/guidance.h
- src/guidance.cpp
- src/main.cpp
- launch/turtlebotDemo.launch

# Dependencies 
- ROS Kinetic 
To install follow this [link](http://wiki.ros.org/kinetic/Installation)
- Ubuntu 16.04
- Turtlebot packages 
To install turtlebot, type the following:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

# Building package 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/saimouli/turtlebot_walker.git
cd ..
catkin_make
```

# Running demo 
## Using roslaunch 
Type the following command in a new terminal:
```
roslaunch turtlebot_walker turtlebotDemo.launch
```

## Using rosrun 
Run roscore in a new terminal 
```
roscore
```
launch turtlebot gazebo simulation 
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
run the turtlebot node by running the following command 
```
rosrun turtlebot_walker turtlebot_walker 
```

# Recording using rosbag files 
Record the rostopics using the following command with the launch file: 
```
roslaunch turtlebot_walker turtlebotDemo.launch record:=true
```
recorded bag file will be stored in the results folder 

To record for a specific time 
```
roslaunch turtlebot_walker turtlebotDemo.launch record:=true secs:=20
```
In the above case rosbag will record for 20 seconds

# Playing bag files 
navigate to the results folder 
```
cd ~/catkin_ws/src/turtlebot_walker/results 
```
play the bag file 
```
rosbag play turtlebotRecord.bag
```
Can verify the published topic by echoing the following topic 
```
rostopic echo /mobile_base/commands/velocity
```

