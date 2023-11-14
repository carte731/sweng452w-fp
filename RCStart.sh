#!/bin/bash

# short-cut runs either simulator or actual hardware

# Grabs the user input parameter
MODE=${1,,}

# The user helper print 
help(){
    echo "Enter either sim or real as program inputs"
    echo
    echo "Sim Mode: Launches Gazebo Simulator with Robot Operating System (ROS)."
    echo "This allows the user to test movement commands that will mirror real life hardware."
    echo "Select different maps by changing the launch file."
    echo
    echo "Real Mode: Launches ROS on real hardware. YahBoom hardware must be used."
    echo "If the user would like to use additional hardware support they must"
    echo "create a new launch file (used the current launch files as a template)."
    echo 

}

# The main function
main(){
    # Souces the config variables, so they can be used by
    # ROS and the controller program
    source ./config.sh

    # Sourcing the global and local development spaces
    source /opt/ros/noetic/setup.bash
    source ./../../devel/setup.bash

    # Allows the user to select sim (Gazebo and WSL2) or real mode (actual hardware)
    if [ "$MODE" == "sim" ]; then
        roslaunch sweng452w sweng452w_SIM_launch.launch 
        export RC_MODE=0
    elif [ "$MODE" == "real" ]; then
        #roslaunch /real_mode/yahboomcar_bringup/launch/driver_bringup.launch &
        roslaunch sweng452w sweng452w_launch.launch
        export RC_MODE=1
    else
        help
    fi
}

main