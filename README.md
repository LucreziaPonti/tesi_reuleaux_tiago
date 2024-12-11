This repository contains all the packages used in the work for the thesis: 
**Reuleaux Optimization of Base Placement for Mobile Robots in a Kitchen Environment**

The work includes the use of the robot TIAGo and the Reuleaux packages for base placement, the simulation has been done on a Ubuntu 20 system with ROS Noetic

!!! these instructions may not be fully complete/comprehensive - highly suggest going over the code and the original/official wiki pages to fully understand !!!

## Installation

1. Install ROS Noetic following the official instructions: https://wiki.ros.org/noetic/Installation/Ubuntu

2. In addition install the following packages (some may not actually be of use ultimately):
    ```
    sudo apt-get install git wget ipython3 python3-catkin-tools python-is-python3 ros-noetic-actionlib-tools ros-noetic-moveit-commander ros-noetic-rviz-visual-tools ros-noetic-base-local-planner ros-noetic-four-wheel-steering-msgs ros-noetic-urdf-geometry-parser ros-noetic-people-msgs ros-noetic-ddynamic-reconfigure ros-noetic-navigation ros-noetic-twist-mux ros-noetic-teb-local-planner ros-noetic-moveit-vsual-tools ros-noetic-rosparam-shortcuts
    ```

3. INSTALL MOVEIT from pre-built binaries :  
    ```
    sudo apt install ros-noetic-moveit
    ```
4. Create a workspace to clone the reposiory into : 
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin build
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    cd src
    git clone https://github.com/LucreziaPonti/tesi_reuleaux_tiago.git 
    catkin build
    ```

## Usage:
The repository contains 5 main folders: 
- Grasp: contains the packages with the gazebo plugin for the fake object recongition and the grasp gazebo plugin
- Reuleaux: contains all the packages of the library reuleaux
- sim_tiago_kitchen: contains the launch, config and rviz files used to easily execute the simulation + the kitchen world and meshes, the reuleaux_bp_to_nav and pick_and_place nodes (created specifically for this simulation)
- tiago: contains all the packages provided by pal robotics to simulate and control the robot
- useful stuff: contains videos, fotos of the simulation and tests done and a few useful commands 
