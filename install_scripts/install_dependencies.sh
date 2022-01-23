#Install dependencies for GAIA-drone-control
#should be run as './install_dependencies.sh', without sudo. 
#You may need to enter the root password multiple times during the execution depending on execution speed (e.g, on Jetsons)

#### Install ros_melodic ==========================================================
# instructions from http://wiki.ros.org/melodic/Installation/Ubuntu

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-ros-base

#conditionally add lines to bashrc if it is not already there
#this is a standard source command for ros, you definitely want this one
if ! grep -Fxq "source /opt/ros/melodic/setup.bash" ~/.bashrc; 
    then echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc; 
fi

#this is a convenience source for the gaia-ws, if you are working with other workspaces as well you may want to drop this
if ! grep -Fxq "source ~/gaia-ws/devel/setup.bash" ~/.bashrc; 
    then echo "source ~/gaia-ws/devel/setup.bash" >> ~/.bashrc; 
fi

#This is a convenience call to change permission on /dev/ttyTHS1 so this command does not have to be run before launching Mavros (or our code that uses Mavros). As a consequence you will have to enter the root password whenever you open the terminal, so you may want to delete it if not using the repo frequently.
if ! grep -Fxq "sudo chmod 666 /dev/ttyTHS1" ~/.bashrc; 
    then echo "sudo chmod 666 /dev/ttyTHS1" >> ~/.bashrc; 
fi

sudo ifconfig enp15s0 mtu 9000

# source ~/.bashrc

sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
# Warning: running 'rosdep update' as root is not recommended.
#   You should run 'sudo rosdep fix-permissions' and invoke 'rosdep update' again without sudo to fix it if you accidentally execute this install script with sudo.

sudo apt-get install -y python3-catkin-tools #fixes catkin command not found
#### end ros_melodic installation =================================================



#install some necessary ros vision tools for our project
sudo apt-get install -y ros-melodic-vision-msgs ros-melodic-vision-opencv ros-melodic-cv-bridge

#install mavros
sudo apt install -y ros-melodic-mavros ros-melodic-mavros-extras
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh


#install some fixes for using ros melodic with Python3
sudo apt-get install -y python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

#install necessary python packages
sudo pip3 install Cython
sudo pip3 install numpy

#appears to be necessary for scipy
sudo apt-get install -y gfortran libopenblas-dev liblapack-dev



#install dependencies for running yolov5 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

cd ~
#instructions from https://medium.com/nerd-for-tech/face-mask-detection-with-nvidia-jetson-nano-yolov5-b66f286f16d4
sudo apt-get install -y libopenblas-base libopenmpi-dev
curl -LO https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl
mv p57jwntv436lfrd78inwl7iml6p13fzh.whl torch-1.8.0-cp36-cp36m-linux_aarch64.whl
sudo pip3 install torch-1.8.0-cp36-cp36m-linux_aarch64.whl
rm torch-1.8.0-cp36-cp36m-linux_aarch64.whl

#attempt to print to confirm success"
sudo python3 -c "import torch; print(torch.cuda.is_available())"

sudo apt install -y libjpeg-dev zlib1g-dev
git clone --branch v0.9.1 https://github.com/pytorch/vision torchvision
cd torchvision/
sudo python3 setup.py install
cd ..

sudo apt-get install -y libfreetype6-dev #must be installed before pillow or causes "the imaging_ft c module is not installed" error for yolo. If not remove ALL instances of pillow and reinstall with 'sudo pip3 install --no-cache-dir pillow'

pip3 install -r ~/gaia-ws/src/GAIA-drone-control/install_scripts/requirements.txt #install with requirements.txt, must be in install_dependencies.sh directory when calling script

#attempt to use actual requirements.txt for now
# sudo pip3 install matplotlib
# sudo pip3 install numpy
# sudo pip3 install opencv-python
# sudo pip3 install Pillow
# sudo pip3 install PyYAML
# sudo pip3 install requests
# sudo pip3 install scipy==1.4.1
# sudo pip3 install torch
# sudo pip3 install torchvision
# sudo pip3 install tqdm
# sudo pip3 install tensorboard
# sudo pip3 install pandas
# sudo pip3 install seaborn
# sudo pip3 install thop

#end yolov5 dependencies ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#Convenience installs to prepare for installing spinnaker:
sudo apt-get install -y libusb-1.0-0


