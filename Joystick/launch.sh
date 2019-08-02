#!/usr/bin/env bash
catkin_make
source ./devel/setup.bash
source ./devel/setup.zsh

chmod +x src/joystick_controls/src/VectorCalcDualStick_node.py
chmod +x src/joystick_controls/src/compass_node.py
chmod +x src/joystick_controls/src/TeensyWriteManual_node.py
chmod +x src/joystick_controls/src/VectorCalcSteeringWheel_node.py

roslaunch joystick_launch.launch
