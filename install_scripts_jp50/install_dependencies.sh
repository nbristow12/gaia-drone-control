#Install dependencies for GAIA-drone-control
#should be run as './install_dependencies.sh', without sudo. 
#You may need to enter the root password multiple times during the execution depending on execution speed (e.g, on Jetsons)

# create virtual environment
#mkdir ~/gaia-feedback-control
#cd ~/gaia-feedback-control
#python3.6 -m venv gaia-fc-env
#cd ~

# install all python modules to virtual environment
#py_env=/home/ffil/gaia-feedback-control/gaia-fc-env/bin/python3
py_env=python3

#upgrade pip, required for installing matplotlib and possibly some others
$py_env -m pip install --upgrade pip

# updating package lists
sudo apt update -y
sudo apt-get update -y

# install ninja for faster compile for source
sudo apt-get install -y ninja-build
$py_env -m pip install ninja

#### Install ros_noetic ==========================================================
# instructions from http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-ros-base

#conditionally add lines to bashrc if it is not already there
#this is a standard source command for ros, you definitely want this one
if ! grep -Fxq "source /opt/ros/noetic/setup.bash" ~/.bashrc; 
    then echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc; 
fi

#this is a convenience source for the gaia-ws, if you are working with other workspaces as well you may want to drop this
if ! grep -Fxq "source ~/gaia-feedback-control/devel/setup.bash" ~/.bashrc; 
    then echo "source ~/gaia-feedback-control/devel/setup.bash" >> ~/.bashrc; 
fi

#This is a convenience call to change permission on /dev/ttyTHS1 so this command does not have to be run before launching Mavros (or our code that uses Mavros). As a consequence you will have to enter the root password whenever you open the terminal, so you may want to delete it if not using the repo frequently.
if ! grep -Fxq "sudo chmod 666 /dev/ttyACM0" ~/.bashrc; 
    then echo "sudo chmod 666 /dev/ttyACM0" >> ~/.bashrc; 
fi

sudo ifconfig enp15s0 mtu 9000

# source ~/.bashrc

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
# Warning: running 'rosdep update' as root is not recommended.
#   You should run 'sudo rosdep fix-permissions' and invoke 'rosdep update' again without sudo to fix it if you accidentally execute this install script with sudo.

sudo apt-get install -y python3-catkin-tools #fixes catkin command not found

# seems to be a dependency only for neotic
sudo bash install_geographiclib_datasets.sh

#### end ros_noetic installation =================================================



#install some necessary ros vision tools for our project
sudo apt-get install -y ros-noetic-vision-msgs ros-noetic-vision-opencv ros-noetic-cv-bridge

#install mavros
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh


#install some fixes for using ros noetic with Python3
sudo apt-get install -y python3-pip python3-yaml
$py_env -m pip install rospkg catkin_pkg

#install necessary python packages
$py_env -m pip install Cython
$py_env -m pip install numpy==1.19.4 #1.19.5 causes issue with matplotlib install

#appears to be necessary for scipy
sudo apt-get install -y gfortran libopenblas-dev liblapack-dev

# install dependecies for GoPro camera using goprocam API
$py_env -m pip install goprocam
sudo apt install ffmpeg

#install dependencies for running yolov5 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# installing torch v1.11
cd ~
sudo apt-get install libopenblas-base libopenmpi-dev libomp-dev
wget https://nvidia.box.com/shared/static/ssf2v7pf5i245fk4i0q926hy4imzs2ph.whl -O torch-1.11.0-cp38-cp38-linux_aarch64.whl
$py_env -m pip install torch-1.11.0-cp38-cp38-linux_aarch64.whl --user
rm torch-1.12.0-cp38-cp38-linux_aarch64.whl

#attempt to print to confirm success"
$py_env -c "import torch; print(torch.cuda.is_available())"


# installing torchvision v0.12 (from source)
sudo apt install -y libjpeg-dev zlib1g-dev
git clone --branch v0.12.0 https://github.com/pytorch/vision torchvision
cd torchvision
$py_env setup.py install --user	
cd ~

sudo apt-get install -y libfreetype6-dev #must be installed before pillow or causes "the imaging_ft c module is not installed" error for yolo. If not remove ALL instances of pillow and reinstall with 'sudo pip3 install --no-cache-dir pillow'

# sudo pip3 install -r ~/gaia-ws/src/GAIA-drone-control/install_scripts/requirements.txt #install with requirements.txt, must be in install_dependencies.sh directory when calling script
#attempt to use actual requirements.txt for now
$py_env -m pip install matplotlib
# $py_env -m pip install numpy
$py_env -m pip install opencv-python
# $py_env -m pip install Pillow
# $py_env -m pip install PyYAML
$py_env -m pip install requests
$py_env -m pip install --upgrade scipy #had an issue with not automatically upgrading, need >=1.4.1 for yolo
$py_env -m pip install tqdm
$py_env -m pip install tensorboard
$py_env -m pip install pandas
$py_env -m pip install seaborn
$py_env -m pip install thop

#end yolov5 dependencies ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#Convenience installs to prepare for installing spinnaker:
# sudo apt-get install -y libusb-1.0-0


