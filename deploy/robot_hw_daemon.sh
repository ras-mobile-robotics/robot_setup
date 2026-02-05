#!/bin/bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Set your specific Domain ID (Update this to match your .bashrc)
source /etc/turtlebot4/setup.bash

# Run the script
/usr/bin/python3 /home/ubuntu/robot_setup/robot_hw_daemon.py
