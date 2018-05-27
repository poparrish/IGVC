#!/usr/bin/env bash
catkin_make
source ./devel/setup.bash

chmod +x src/navigation_launch/cameras.py
chmod +x src/navigation_launch/gps.py
chmod +x src/navigation_launch/guidance.py
chmod +x src/navigation_launch/lidar.py
chmod +x src/navigation_launch/nav.py
chmod +x src/navigation_launch/Control_Node.py

roslaunch navigation_launch.launch
