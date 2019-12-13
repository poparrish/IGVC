# Development

## IDE

An IDE is not a requirement but highly recommended for ROS 
development. There are a variety of free Python and C++ IDEs 
available for educational use. Many of us have used JetBrains 
IDEs (namely PyCharm and CLion) for developing this software.

JetBrains software is free to use for students. You can apply 
for a student license [here](https://www.jetbrains.com/student/).
All you need to do is provide your boisestate.edu email address.

### JetBrains IDE Setup

Some setup is required to get JetBrains IDEs to work with ROS.
Assuming you have the project running from the command line, you
can do the following:

1.) Create a [launch script](https://www.jetbrains.com/help/idea/working-with-the-ide-features-from-command-line.html#launchers-macos-linux).
Place it wherever you want.

2.) Source the current ROS environment:

```bash
cd ~/IGVC/Navigation-Launch
source devel/setup.bash
```

3.) Launch the IDE

```bash
{script-location}/clion.sh
```

You should now get autocomplete features for all ROS libraries.

## Build

To build 

## Run

We use the `roslaunch` utility to bring up the project. XML files
are used to configure the environment and start up all nodes. 

### Dev

To start up a development environment:

```bash
roslaunch navigation_launch dev.launch
```

One of the easiest ways to develop a ROS node is to add
all of its dependencies to the `dev.launch` file and launch
it once. Then, you can build and run your node separately.
This allows for much faster turnaround time.

```bash
catkin_make
./devel/lib/bender/my-node
```

### Full

To start everything (requires all sensors to be connected):

```bash
roslaunch navigation_launch navigation_launch.launch
```
