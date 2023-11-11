# Penn State SWENG452W Final Project
This repo contains the final project for SWENG-452W. I created a Remote Controller (RC) car running on a Raspberry Pi 4 that can be controlled via a Ground Control Station (GCS) on another computer. The Robot Operating System (ROS) was selected as the main tool for interacting with the hardware - due to it being the industry standard.

There are two modes for this software: simulation (sim) and real hardware (real) modes. This README will cover and provide resouces on how to set-up both environments and the GCS for testing and/or operations.

## Simulation-Mode Set-Up
The sim-mode allows for development and testing of the remote control system without having to mount to actual hardware. This allowed me to save time and money in development. Currently the project uses [Gazebo](https://gazebosim.org/home) as the simulator of choice and [Windows System for Linux 2](https://learn.microsoft.com/en-us/windows/wsl/about) (WSL2) to run ROS and Gazebo for simulator operations. This section will cover how to set-up a simulator environment and the GCS.

### WSL2
WSL2 was chosen over a standard virtual machine due to the face GPU bypassing is built in the platform. This allows you to use anything with a Graphical User Interface (GUI) natively - additionally, it allowed me connect, develop and cross-compile on [Visual Studio Code](https://code.visualstudio.com/docs/remote/wsl) on my Windows host machine. WSL2 running Ubuntu 20 will be installed.

#### Installing WSL2-Base
Open PowerShell in admin mode. This can be done by searching for `powershell` in the Windows search bar. Next, right click the serach result and select `Run as administrator`.

![PowerShell Admin Mode](README_Supps/image.png)

Next type the text below into the powershell to install base WSL2 onto your computer. 
``` powershell
wsl --install
```

#### Install WSL2 Ubuntu 20
In the Windows search bar, type `windows store`. In the Windows store search for `Ubuntu 20` and select `Ubuntu 20.04.6 LTS` - then install it.

![WSL2-Ubuntu20](README_Supps/wsl2U.png)

#### Install ROS on WSL2-Ubuntu 20
For this section, you can install ROS in you new WSL2-Ubuntu 20 environment using two methods. 

##### Install ROS, ROS workspace and Project automatically using Installation BASH script
The installation bash script in the main portion of this repo will automate the installation of ROS, workspace creation and install the ROS project/packages. **It's recommended to manually install, so you can understand the process.** If you run this script `installROS.sh`, there is no need to do the following sub-sections. Use the command below in the main directory (assuming you git cloned the directory into WSL2).
``` bash
./installROS
```
##### Install ROS and Project Manually
The manual sub-section is in two parts, installing ROS, setting up ROS workspace and installing the final project. This will be done in the WSL2-Ubuntu 20 environment.

###### ROS Installation
Here are the instructions for ROS installation - if you would like for indepth information follow [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu).

Setup ROS sources list
``` bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" 
> /etc/apt/sources.list.d/ros-latest.list'
```
Install Curl for keys download
``` bash
sudo apt install curl -y
```
Install GIT so you can download final project from GitHub
``` bash
sudo apt install git -y
```
Update Keys
``` bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
Update you system
``` bash
sudo apt update
```
Install ROS version: Noetic
``` bash
sudo apt install ros-noetic-desktop-full
```
Add the ROS workspace sourcing to your bashrc,
Will automatically start ROS on terminal start-up. 
``` bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
Source (restart) your bash environment to start ROS. This will happen automatically in the future.
``` bash
source ~/.bashrc
```
Install Python dependencies
``` bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```
Start the Python ROS-Dep
``` bash
sudo rosdep init
```
Update ROS-Dep
``` bash
rosdep update
```
###### Creating a ROS Workspace
In this sub-section, we will create a ROS workspace to develop and program ROS programs, packages and projects. In you home directory in WSL2, run this command to create the directories.
``` bash
mkdir -p ~/catkin/src
```
Next, we will initialize the ROS workspace.
``` bash
cd ~/catkin/src
catkin_init_workspace
```
Now go back to the main work directory and do the initial make in the ROS workspace.
``` bash
catkin_make
```

###### Final Project Installation
In this sub-section we will set up and installation of the final project. In WSL2 environment go into you new ROS workspace
Now lets git clone the final project into the ROS workspace
``` bash
cd ~/catkin/src
``` 
Now GIT clone the repo into you source code section.
``` bash
git clone https://github.com/carte731/sweng452w-fp.git
```
Now initialize the sub-module git repos **(This step is not nesscessry if you're not running on real hardware aka. moving a real RC car/robot).**
``` bash
cd sweng452w-fp
git submodule init
``` 
Next, we will download the simulator repos **(This step is not nesscessry if you're not running on real hardware aka. moving a real RC car/robot).**
``` bash
git submodule update --recursive
```
After going back to the main ROS workspace. We will download all the require project dependcies using this command.
``` bash
cd ~/catkin/
rosdep install --from-paths src/ -y
```
Next install the hardware drivers **(This step is not nesscessry if you're not running on real hardware aka. moving a real RC car/robot).**
``` bash
cd /src/sweng452w-fp/real_mode/Driver_installer
sudo python3 setup.py install
```
Finally, make your ROS workspace
``` bash
cd ~/catkin/
catkin_make
```

## Real-Mode Set-Up
For this project, I used [YahBoom](http://www.yahboom.net/home) for my robot car chassis. They provide an assortment of chassis, boards and accessories robotics design.

### Chassis
I chose the [Yahboom Aluminum Alloy ROS Robot Car Chassiss](https://category.yahboom.net/products/ros-chassis?_pos=1&_sid=4b44cf093&_ss=r&variant=44178906939708). It's a basic chassis, with just wheels and a mounting frame - the final project I created provides the drive control, so the more complex platforms were not needed. Here's a tutorial showing how to assemble the chassis: [4WD chassis Installation Video](https://youtu.be/sRYrv7y6zoY?si=ojyGAra0tV3s3jSz)

![4WD chassis](README_Supps/4WD.png)

### Hardware Control Board
The control board interfaces with the hardware and allows communication with the hardware through a centerized system. I chose the [YahBoom ROS Robot Control Board](https://category.yahboom.net/products/ros-driver-board?_pos=1&_sid=f40392f38&_ss=r) - my program interfaces with the board to send drive commands to the hardware.

![STM32F103RCT6 IMU for Raspberry Pi Jetson Robotics](README_Supps/ROSControl.png)

### Assembly
The Installation video provided above will show you have to assemble the chassis. The [ROS installation sub-section](#Final-Project-Installation) above will install the hardware controller drivers. 

For further details, please read the material below:
- [ROS Robot chassis repository](http://www.yahboom.net/study/ROS-chassis)
- [ROS robot expansion board repository](http://www.yahboom.net/study/ROS-driver-Board)

## GCS Set-Up
The GCs send commands to the RC car/robot from another computer or VM running Linux (Generally Ubuntu 20, but should be able to run on any OS. Other OS's haven't been tested). It runs off a GUI, that requires [ QT Creator](https://www.qt.io/product/development-tools) to run. The GCS runs off of Qt Creator 4.5.2 (later versions have not been tested).

![Alt text](README_Supps/GCSMain.png)

## GCS Controls

### RC Controls and Execution Mode
There are 

### Set-Up

#### Setting up GCS
In a virual machine (VM) environment of your choice create or use a Ubuntu/Linux environment. Change the network type to `bridged` for that VM.

![Alt text](README_Supps/bridged.png)

## Running Sim and Real Modes
### Running Simulation-Mode
In the main directory there are two files: `config.sh` and `RCStart.sh`. Config file **must** be set

### Running Real-Mode


