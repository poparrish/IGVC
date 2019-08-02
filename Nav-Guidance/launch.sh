#!/usr/bin/env bash
catkin_make
source ./devel/setup.bash

roslaunch navigation_launch navigation_launch.launch
