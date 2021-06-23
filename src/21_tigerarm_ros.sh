#!/bin/bash
gnome-terminal -- bash -c "roslaunch pixle_godzilla_moveit_config demo.launch"
sleep 7
gnome-terminal -- bash -c "roslaunch arm_moveit_kinematics demo.launch" &
wait
exit 0
