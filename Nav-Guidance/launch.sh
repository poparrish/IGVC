#!/usr/bin/env bash
source ./devel/setup.bash

chmod +x src/navigation_launch/cameras.py
chmod +x src/navigation_launch/gps.py
chmod +x src/navigation_launch/guidance.py
chmod +x src/navigation_launch/lidar.py
chmod +x src/navigation_launch/nav.py
chmod +x src/navigation_launch/control.py

roslaunch navigation_launch.launch
