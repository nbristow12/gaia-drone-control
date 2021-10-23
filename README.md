# GAIA-drone-control

gaia-drone-control includes control and simulation methods developed for controlling Arducopter via Mavros for the GAIA project. Designed for use with ROS Melodic on Ubuntu 18.04 and Arudcopter 4.0.7.

## Installation

Clone this package into your catkin_ws/src folder, and use catkin build or catkin_make as appropriate to build the ROS nodes.

Environment setup instructions for ROS, Arucopter SITL, and other depencdencies still to come.

## Demonstrations


### Basic Arducopter SITL Startup Example w/ Mavros telemetry

In one terminal:
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```
In a second terminal:
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

Launch a mavros instance in a third terminal:
```bash
cd ~/catkin_ws/src/GAIA-drone-control/launch
roslaunch mavros-telem.launch
```

The mavros telemetry can then be viewed in a fourth terminal. First verify publishing rate with rostopic hz, then see contents with echo:
```bash
rostopic hz /mavros/local_position/pose
ctrl-c
rostopic echo /mavros/local_position/pose
```

This completes the drone startup,
\
### Manual Drone Control Using Arducopter SITL Terminal
The following commands can be entered in the arducopter terminal (second terminal from instructions above) to control the vehicle:
#### Takeoff:
```bash
GUIDED
arm throttle
takeoff 10
```
This places the drone into guided mode (so it can accept computer control), arms it, and tells it to take off to 10m. 
\
\
After takeoff the "position x y z" or "velocity vx vy vz" commands can be used to move the drone. For position x,y, and z are relative distances to move  and in north, east, down coordinates, e.g. position 1 0 -1 would move 1m forward and 1m up. The "help" command lists these and other commands, and typing most commands such as "position" or "velocity" without arguments provides some basic instructions on how to use them.