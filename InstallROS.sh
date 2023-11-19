#!/bin/bash

### INSTALL ROS-NOETIC ###

# Setup your sources.list - Setup your computer to accept software from packages.ros.org.
echo -n "\nSetting up source listing to accept ROS packages\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
# if you haven't already installed curl
echo -n "\nInstalling Curl\n"
sudo apt-get install curl -y

# Installing Git
echo -n "\nInstalling GIT version controller\n"
sudo apt-get install git -y

# Installing Pyton is python (links python to python3)
echo -n "\nInstalling Python-is-Python3 \n"
sudo apt install python-is-python3 -y

# Updating key library
echo -n "\nUpdating libraries for ROS\n"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Updaing Linux after library update 
echo -n "\nUpdaing Linux...\n"
sudo apt-get update

# Installing ROS-Noetic
echo -n "\n Installing ROS-Noetic\n"
sudo apt-get install ros-noetic-desktop-full -y

# Setting up Environment
echo -n "\nAdding the source command to .bashrc file\n"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dependencies for building packages
echo -n "\nInstalling Python ROS dependencies\n"
sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# Initialize rosdep
echo -n "\nInstalling Python-ROS\n"
sudo apt-get install python3-rosdep -y

# Update Ros-Dep for python
echo -n "\nUpdaing Python-ROS\n"
sudo rosdep init
rosdep update

### SETTING UP ROS WORKSPACE ###

# Making ROS operations directory
echo -n "\n Creating ROS workspace directory structure\n"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Initalizing workspace
echo -n "\nInitializing ROS workspace\n"
catkin_init_workspace 

# Building workspace
echo -n "\nBuilding ROS workspace\n"
cd ..
catkin_make

### INSTALLING PROJECT ###

# Copying workspace 
echo -n "\nGIT cloning an additional copy of SWENG project to new ROS workspace\n"
cd ~/catkin_ws/src
git clone https://github.com/carte731/sweng452w-fp.git

# Grabbing submodules in project GIT repo
echo -n "\nCloning git submodules (Used for Simulator-Mode)\n"
cd sweng452w-fp
git submodule init
git submodule update --recursive

# Installing package dependencies
echo -n "\nInstalling SWENG ROS project dependencies\n"
cd ../../
rosdep install --from-paths src/ -y

# Install Real-Time mode drivers for hardware
echo -n "\nInstall YahBoom hardware drivers (Used for Real-Time Mode)\n"
cd src/sweng452w-fp/real_mode/Driver_installer
sudo python3 setup.py install
# Returning to main dir for ROS workspace
cd -

# Build with the new ROS workspace
echo -n "\nBuilding the new ROS workspace with SWENG project files\n"
catkin_make

### EXIT MESSAGE ###

# Exit message
echo -n "\n\nPROJECT INSTALL COMPLETE..."
echo 
echo "Use ./RCStart.sh in '~/catkin/src/sweng452w-fp' to run RC vehicle in sim or real mode."
echo 
echo -n "read the README for more info.\n\n"