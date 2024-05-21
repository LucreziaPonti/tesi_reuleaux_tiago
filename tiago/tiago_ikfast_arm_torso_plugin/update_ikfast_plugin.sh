search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=tiago.srdf
robot_name_in_srdf=tiago
moveit_config_pkg=tiago_moveit_config
robot_name=tiago
planning_group_name=arm_torso
ikfast_plugin_pkg=tiago_ikfast_arm_torso_plugin
base_link_name=torso_lift_link
eef_link_name=arm_tool_link
ikfast_output_path=/home/lucry_wsl_20/tesi_ws/src/tesi_reuleaux_tiago/tiago/tiago_ikfast_arm_torso_plugin/include/tiago_ikfast_arm_torso_plugin/src/tiago_arm_torso_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
