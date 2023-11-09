#!/bin/bash

## Config file for program
##    -This is the config file for:
##        IP address and port number of Ground Control Station (GCS)
##        The RC car server port number
##        The vehicle type: burger, waffle or waffle_pi

## GCS ##

## GCS IP Address ##
export GCS_IPADDR="10.0.0.101"

# GCS Port Address
export GCS_PORT=9000

## RC IN-BOUND COMMAND SERVER ##

# RC Server Command Port
# The server port that will receive inbound commands
export CMD_PORT=3390

# Select which simulation RC vehicle to use;
# Options: burger, waffle, waffle_pi
export TURTLEBOT3_MODEL=burger