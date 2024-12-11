# **TIAGo packages**

This folder contains all the packages provided by pal robotics used to simulate the robot tiago. 

### Installation and use

Follow the installation instructions and tutorials at: https://wiki.ros.org/Robots/TIAGo/Tutorials 


### Added packages
Other than the packages given by pal ro botics the tiago_ikfast_arm_plugin has been added.
This package was created using OpenRAVE's IKFast solver for TIAGo (generated separately on Ubuntu16 system)

## Useful commands: 
Simple simulation launch:
```
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper
```
Tiago with moveit launch:
```
roslaunch tiago_moveit_config demo.launch
```
Tiago with navigation stack (use the rviz button) [for more detailed instructions see tutorial in ros wiki and raccolta_comandi_navigazione_tiago.txt in "useful stuff" folder]: 
```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=*path to map folder*
```

### Control of tiago (to use with simulation already started):
Key teleop: 
```
rosrun key_teleop key_teleop.py
```

PLAY MOTIONS:
```
rosparam list | grep  "play_motion/motions" | grep "meta/name" | cut -d '/' -f 4
rosrun play_motion run_motion *motion name*
```
Joint control: 
```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```