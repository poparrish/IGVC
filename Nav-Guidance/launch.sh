#!/usr/bin/env bash
catkin_make
source ./devel/setup.bash

chmod +x src/navigation_launch/cameras.py
chmod +x src/navigation_launch/gps.py
chmod +x src/navigation_launch/odometry.py
chmod +x src/navigation_launch/guidance.py
chmod +x src/navigation_launch/lidar.py
chmod +x src/navigation_launch/nav.py
chmod +x src/navigation_launch/mapping.py
chmod +x src/navigation_launch/mapping_debug.py
chmod +x src/navigation_launch/potential_field.py
chmod +x src/navigation_launch/Control_Node.py
chmod +x src/navigation_launch/TeensyWrite_node.py
chmod +x src/navigation_launch/compass_node.py

roslaunch navigation_launch navigation_launch.launch
