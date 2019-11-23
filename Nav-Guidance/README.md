# Navigation Module

### Project Overview

All of our code is written using either C++ or Python. We are trying to
move C++ for CPU-intensive code.

Our code is located in the `src/navigation` package. Other external
packages that are needed are in the `src` folder.



### Setup

1.) Clone submodules

External libraries and ROS nodes are referenced with git submodules. 
Run `git submodule update --init --recursive` to make sure you have them
checked out.

2.) Initialize ROS workspace

To setup the ros workspace, run `catkin_init_workspace` in the
`Nav-Guidance` folder. 

3.) Build ROS nodes
### 

Then, run `catkin_make` to build all of the modules.

### (Optional) CLion Setup

First, create a
[command line launch script](https://www.jetbrains.com/help/idea/working-with-the-ide-features-from-command-line.html#launchers-macos-linux) 
for CLion. 

Then, source the current workspace.

`source devel/setup.bash`

Finally, launch CLion using the script you generated. Code completion
and highlighting should work correctly for Python and C++.
