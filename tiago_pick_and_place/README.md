## TIAGo pick and place for reuleaux
====

#1. Pick and place server
Makes the actions "pick_pose" and "place_pose" available for use 
Both are of type PickUp.action: 
  - goal: geometry_msgs/PoseStamped object_pose , string object_name, string surface
  this allows the most flexible use of the pick and place actions of moveit by giving infos on which object, in which position, and on which surface we want to move
  - result: int32 error_code 
  Comes from moveit_msgs/MoveItErrorCodes

#2. Pick and place handler
Takes as parameters the yaml files containing the grasping task we want to execute:
  - pick tasks and place tasks are the yaml files we used with the base placement plugin
  - tiago poses are the poses on the "back" of tiago we can use to move around multiple objects using tiago as a "shelf"
Creates the collision objects for the planning scene: creates immediately the objects in the scene and the 2 tables, which are fixed. 
It also creates the collision object for the tiago back, attached to tiago "torso_lift_link"
The 2 tables and tiago are the "surfaces" which are called: table_pick (where the objects originally are - the table), table_place (where we want to put them - for now the sink), tiago_back

It makes avaliable 3 services: 
  - pick_by_name: simply calls the pick action but only requires the name of the object we want to pick up
    request: string object_name (name of the model in gazebo), bool tiago (true=pick from tiago_back, false pick from table_pick) -- responce int32 error_code
  - place_by_name: same but for placing the object down
  request: string object_name (name of the model in gazebo),bool tiago (true=place on tiago_back, false=place on table_place) -- responce int32 error_code
  - move_all: iteratively calls the pick and place actions to move all the objects form the pick table to tiago or from tiago to the place table, depending on the request
  request: bool pick (true= pick from table_pick, place on tiago / false= pick from tiago, place on table_place)

#2.1. intended use for the services
The *_by_name services should be used if we want to move one object at a time. This way TIAGo has to make multiple trips between table and sink/dishwasher. 
So we would move tiago with the navigation to the table, call the pick_by_name service, move tiago to the sink, call the place_by_name service.
!!! in this case we could (/should?) redo the base placement every time we move, since the tasks change


The move_all service allows to move tiago to the table once, pick all the objects ("loading" them on tiago's back), move tiago to the dishwasher/sink (carefully so that it doesn't make the objects fall), place all the objects ("unload" tiago)
!! this allows to use the base placement only twice, for the loading phase and for the unloading phase

3#. Launch files
pick_and_place.launch allows to start rviz with everything we need, but requires the simulation to be launched separately 
there are 3 arguments to specify the yaml files where the manipulation poses are specified (without the ".yaml"): 
 - pick_yaml: pick grasping poses - the yaml file is the "saved_task" folder of the base_placement_plugin pkg - default= pick_on_table_simple_manuale
 - place_yaml: place poses - the yaml file is the "saved_task" folder of the base_placement_plugin pkg - default= place_2sink_2plane
 - tiago_yaml: poses to load/unload the objects on tiago's back, specified WRT torso_lift_link frame, stored in the "config" folder of this pkg - default= tiago_poses_2



