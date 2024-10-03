cmake_minimum_required(VERSION 2.8.3)
project(tiago_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  roscpp
  moveit_ros_planning_interface
  
  gazebo_ros
  std_msgs
  std_srvs
)



add_action_files(
  DIRECTORY action
  FILES
  PickUpPose.action
)
add_service_files(
  FILES
  PickUpName.srv
  MoveAll.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

# Add your executable
add_executable(pick_place_server src/pick_place_server.cpp)
# Link against the required libraries
target_link_libraries(pick_place_server  ${catkin_LIBRARIES})
# Install targets
install(TARGETS pick_place_server  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Add your executable
add_executable(pick_place_handler src/pick_place_handler.cpp)
# Link against the required libraries
target_link_libraries(pick_place_handler  ${catkin_LIBRARIES} )
# Install targets
install(TARGETS pick_place_handler  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

