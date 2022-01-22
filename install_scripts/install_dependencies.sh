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
if ! grep -Fxq "source /opt/ros/melodic/setup.bash" ~/.bashrc; 
    then echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc; 
fi

if ! grep -Fxq "source ~/gaia-ws/devel/setup.bash" ~/.bashrc; 
    then echo "source ~/gaia-ws/devel/setup.bash" >> ~/.bashrc; 
fi

#TODO: Add similar command for the chmod command
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

#install some fixes for using ros melodic with Python3
sudo apt-get install -y python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

#install necessary python packages
sudo pip3 install numpy #add more packages here



#install dependencies for running yolov5 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# pip3 install -r requirements.txt #install with requirements.txt, must be in install_dependencies.sh directory when calling script

cd ~
#instructions from https://medium.com/nerd-for-tech/face-mask-detection-with-nvidia-jetson-nano-yolov5-b66f286f16d4
sudo apt-get install libopenblas-base libopenmpi-dev
curl -LO https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl
mv p57jwntv436lfrd78inwl7iml6p13fzh.whl torch-1.8.0-cp36-cp36m-linux_aarch64.whl
sudo pip3 install torch-1.8.0-cp36-cp36m-linux_aarch64.whl
rm torch-1.8.0-cp36-cp36m-linux_aarch64.whl

#attempt to print to confirm success"
sudo python3 -c "import torch; print(torch.cuda.is_available())"

sudo apt install libjpeg-dev zlib1g-dev
git clone --branch v0.9.1 https://github.com/pytorch/vision torchvision
cd torchvision/
sudo python3 setup.py install
cd ..

#mattempt to use actual requirements.txt for now
pip3 install matplotlib
pip3 install numpy
pip3 install opencv-python
pip3 install Pillow
pip3 install PyYAML
pip3 install requests
pip3 install scipy
pip3 install torch
pip3 install torchvision
pip3 install tqdm
pip3 install tensorboard
pip3 install pandas
pip3 install seaborn
pip3 install thop

#end yolov5 dependencies ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#TODO:
#Add .bashrc convenience scripts like sudo chmod 666 /dev/ttyTHS0 (will require a root password whenever opening terminal) and 'source !/gaia-ws/devel/setup.bash'