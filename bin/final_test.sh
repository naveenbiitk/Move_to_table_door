#!/bin/bash
cd ~/catkin_ws/src/find_fridge/bin

stretch_robot_home.py

roslaunch find_fridge aruco_test.launch

cd ~/catkin_ws/src/stretch_web_interface/bash_scripts/
./start_web_server_and_robot_browser.sh
