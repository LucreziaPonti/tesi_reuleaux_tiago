#!/bin/bash

if [ "$1" != "" ]; then
  echo "hello"
  echo "$PWD/$1"

  
  roslaunch "$1" planning_context.launch load_robot_description:=true end_effector:=pal-gripper 
  roslaunch "$1" move_group.launch load_robot_description:=true allow_trajectory_execution:=true end_effector:=pal-gripper fake_execution:=true info:=true

else 
  echo "$0: Please provide a moveit package for your robot:"
fi
