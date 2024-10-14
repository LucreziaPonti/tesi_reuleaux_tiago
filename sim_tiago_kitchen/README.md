# **IntelliMan - Use Case 2 Scenario - Gazebo Models**

This project is devoted to the simulated scenario in Gazebo of the Kitchen available at Eurecat (EUT), developed for the Use Case 2 within the Intelliman project. The examples given consider the use of TIAGo robotic platform. Right now it has been tested with **ROS Noetic** (**Ubuntu 20.04**).

The project is structured as follows:

- Object meshes are located in the *meshes* folders.

- The map of the kitchen is located in the *maps* folder.

- Inside the fodler *urdfs* we can find the urdf of the final sink furniture and the corresponfing *sdf* for their use in Gazebo.

- Worlds are located in the *worlds* folder.

- Inside *utils* you can find a file with all the instructions to run the simulation without any explanation and a folder for the use of docker compose or devcontainer (VSCode) to start the docker container.


### Installation

The project has to be installed within an existing docker image. For the TIAGo case, using the exitsing noetic image:
```
docker pull palroboticssl/tiago_tutorials:noetic
```
A docker container has to be started. This could be done using the available example in [utils](https://dei-gitlab.dei.unibo.it/asanmiguel/intelliman-uc2-gazebo-models/-/blob/main/utils/docker_compose_devcontainer/docker-compose.yaml), where **the path to package has to be modified to fit your configuration**. This container considers CPU usage, for GPU usage (Nvidia) uncomment `runtime:nvidia` and `- NVIDIA_VISIBLE_DEVICES=all`. 


### Simulated scenario

If you do not have GPU you need to export this environment variable:
```
export LIBGL_ALWAYS_SOFTWARE=1
```
Build the package:
```
catkin build sim_tiago_kitchen
```
Also remember to source the workspace:
```
cd CATKIN_WS
source devel/setup.bash
```
To start the simulation for MAPPING you need to run the command:

```
roslaunch sim_tiago_kitchen tiago_mapping.launch public_sim:=true
```
To start the simulation for NAVIGATION, TESTING, or DEVELOPMENT you need to run the command:

```
roslaunch sim_tiago_kitchen tiago_navigation.launch map:=/tiago_public_ws/src/sim_tiago_kitchen/maps/eut_kitchen public_sim:=true
```

It is mandatory to define the public sim arguement.

Additional arguments can be used (see launch file for more details)

