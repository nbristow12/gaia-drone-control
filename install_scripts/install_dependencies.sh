#Install dependencies for GAIA-drone-control
#should be run as './install_dependencies.sh', without sudo. 
#You may need to enter the root password multiple times during the execution depending on execution speed (e.g, on Jetsons)

#### Install ros_melodic ==========================================================
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-ros-base

#conditionally add line to bashrc if it is not already there
if ! grep -Fxq "source /opt/ros/melodic/setup.bash" ~/.bashrc; 
    then echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc; 
fi
# source ~/.bashrc

sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
# Warning: running 'rosdep update' as root is not recommended.
#   You should run 'sudo rosdep fix-permissions' and invoke 'rosdep update' again without sudo to fix it if you accidentally execute this install script with sudo.

sudo apt-get install -y python3-catkin-tools
#### end ros_melodic installation =================================================

#install some necessary ros vision tools for our project
sudo apt-get install -y ros-melodic-vision-msgs ros-melodic-vision-opencv ros-melodic-cv-bridge

#install mavros
sudo apt install -y ros-melodic-mavros ros-melodic-mavros-extras

#install some fixes for using ros melodic with Python3
sudo apt-get install -y python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg