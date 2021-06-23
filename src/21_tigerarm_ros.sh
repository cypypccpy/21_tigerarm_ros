#!/bin/bash
gnome-terminal -x bash -c "roslaunch pixle_godzilla_moveit_config demo.launch"
sleep 4
gnome-terminal -x bash -c "roslaunch arm_moveit_kinematics demo.launch" &
wait 
exit 0
