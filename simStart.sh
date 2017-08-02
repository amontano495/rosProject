#!/bin/bash

gnome-terminal --working-directory="/home/amontano/ardupilot/APMrover2" -e "sim_vehicle.py -j4 -L Quad --console --map"
sleep 5
gnome-terminal --working-directory="/home/amontano/ardupilot_ws/src" -e "roslaunch apm.launch"
