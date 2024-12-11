These packages allow to use gazebo plugins for:
- fake object recognition: collect information of gazebo objects (see *gazebo-pkgs*) and use it to create the planning scene (see *moveit-pkgs*)
- object grasp: in the tiago.urdf.xacro (in tiago/tiago_robots/tiago_description/robots folder) the plugin has already been added (for the parallel gripper), this plugin allows objects in gazebo to "stick" to the gripper whenever a force is applied between the object and the gripper.
    With tiago use the services /parallel_gripper_controller/grasp and /parallel_gripper_controller/release to test it

Highly suggest reading into the README files and wikis of each package for deeper understanding