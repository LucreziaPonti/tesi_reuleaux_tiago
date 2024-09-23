/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2016, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_arm_fk");

  if ( argc < 8 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun tiago_moveit_tutorial plan_arm_fk arm_1 arm_2 arm_3 arm_4 arm_5 arm_6 arm_7");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments are the target values for the given joints");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }

  std::map<std::string, double> target_position;

  target_position["arm_1_joint"] = atof(argv[1]);
  target_position["arm_2_joint"] = atof(argv[2]);
  target_position["arm_3_joint"] = atof(argv[3]);
  target_position["arm_4_joint"] = atof(argv[4]);
  target_position["arm_5_joint"] = atof(argv[5]);
  target_position["arm_6_joint"] = atof(argv[6]);
  target_position["arm_7_joint"] = atof(argv[7]);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm("arm");
  //choose your preferred planner
  group_arm.setPlannerId("SBLkConfigDefault");

  arm_joint_names = group_arm.getJoints();

  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < arm_joint_names.size(); ++i)
    if ( target_position.count(arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << arm_joint_names[i] << " goal position: " << target_position[arm_joint_names[i]]);
      group_arm.setJointValueTarget(arm_joint_names[i], target_position[arm_joint_names[i]]);
    }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm.setPlanningTime(5.0);
  bool success = bool(group_arm.plan(my_plan));

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e = group_arm.move();
  if (!bool(e))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}
