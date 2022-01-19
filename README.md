# GAIA-drone-control

gaia-drone-control includes control and simulation methods developed for controlling Arducopter via Mavros for the GAIA project. Designed for use with ROS Melodic on Ubuntu 18.04 and Arudcopter 4.0.7.

## Installation

If you don't already have a catkin_ws folder you prefer to use, folder, and use catkin build or catkin_make as appropriate to build the ROS nodes.

See below for environment setup instructions for ROS, Arucopter SITL, and other dependencies.

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

## Environment Setup

Designed to be installed on a Jetson mini running Ubuntu 18.04. Can also be installed on WSL or Virtualbox.

### Step 1: Ros Melodic/Mavros/Gazebo Installation:
Follow the install instructions for ROS Melodic on the ROS wiki. 
http://wiki.ros.org/melodic/Installation/Ubuntu

Choose 'sudo apt install ros-melodic-desktop' when choosing how many of the resources to install, this includes the necessary and useful simulation packages without using as much storage space as the desktop-full option.

After completing these steps ensure the 'catkin' command is recognized. If it is not, install with the following command:
```bash
sudo apt-get install python3-catkin-tools
```

Install necessary computer vision tools:
```bash
sudo apt-get install ros-melodic-vision-msgs ros-melodic-vision-opencv ros-melodic-cv-bridge
```

Install mavros using the following command:
```bash
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```
Install Gazebo:
```bash
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

### Step 2: Arducopter SITL installation
Overarching tutorial:
https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
From there first follow link to:
https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

#### Clone repo, checkout 4.0.7 release
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 0bb18a153c
git submodule update --init --recursive
```
#### Install requirements
Use tools script included with the repo
```bash
cd ~/arducopter
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Reload the path (will become permanent when device is restarted)
```bash
. ~/.profile
```
#### Build Arducopter
Instructions at https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md
Use board type 'sitl' (software in the loop), which is also the default
```bash
./waf configure --board sitl
./waf copter
```
From here the build is complete and should be ready to follow the demo tutorials above.

#### Update notes:
Should finish updating this readme to use the install script instead.

Add a few lines of bash commmands similar to the following:
Create gaia-ws folder
clone repo
install using install script
go to gaia-ws, catkin init
catkin build (this sets up ros paths)
source ~/gaia-ws/devel/setup.bash (optionally add this to bashrc for convenience)