#!/bin/bash

echo "##########################"
echo "### Permission Setting ###"
echo "##########################"

sudo chmod a+rw /dev/bus/usb/001/*


echo "##################"
echo "### ros_launch ###"
echo "##################"
roslaunch ricoh_theta_viewer launch.launch
