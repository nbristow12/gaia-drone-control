# GAIA-drone-control

gaia-drone-control includes control and simulation methods developed for controlling Arducopter via Mavros for the GAIA project. Designed for use with ROS Melodic on Ubuntu 18.04 and Arudcopter 4.0.7.

## Installation

Clone this package into your catkin_ws/src folder, and use catkin build or catkin_make as appropriate to build the ROS nodes.

Environment setup instructions for ROS, Arucopter SITL, and other depencdencies still to come.

## Demonstrations


### Basic Arducopter SITL Example

In one terminal:
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```
In a second terminal:
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```