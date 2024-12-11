# **sim_tiago_kitchen**

This package contains all the launch and config files used for the final simulation for the thesis. 

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
Additional arguments can be used (see launch file for more details)

## Full simulation launch

- Launch the simulation, with gazebo object and grasp plugins, navigation and control:
    ```
    roslaunch sim_tiago_kitchen tiago_sull_sim.launch
    ```

- After everything is fully loaded, create the planning scene in rviz (using the fake object recongition): 
    ```
    roslaunch sim_tiago_kitchen planning_scene_kitchen.launch
    ```

- To use the reuleaux base placement plugin (and reuleaux_bp_to_nav): 
    ```
    roslaunch base_placement_plugin rviz_bp.launch
    ```
    To move to the obtained results (for PCA,IKS,GRS methods) : 
    ```
    rosservice call reuleaux_bp_to_nav/move_to_bp_arm
    ```
    To move to the obtained results (for VRM,UI methods) : 
    ```
    rosservice call reuleaux_bp_to_nav/move_to_bp_robot
    ```

- To execute the pick and place: 
    !! as of now the tasks have to be manually inserted in the tiago_pick_and_place.cpp node by copy-paste (before starting, copy the tasks form the .yaml file in saved tasks into the node, then build)
    ```
    roslaunch sim_tiago_kitchen tiago_pick_and_place.launch
    ```
    to actually execute a pick : 
    ```
    rosservice call /tiago_pick_and_place/pick *object name* *do_pregrasp: true/false*
    ```
    to actually execute a place : 
    ```
    rosservice call /tiago_pick_and_place/place *object name* *do_pregrasp: true/false*
    ```