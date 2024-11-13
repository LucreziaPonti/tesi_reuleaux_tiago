#include <ros/ros.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/PlaceGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <tiago_pick_and_place/PickUpPoseAction.h>


class PickAndPlaceServer {
public:
    PickAndPlaceServer() 
    : move_group("arm")
    {
        ROS_INFO("Initializing PickAndPlaceServer...");

        // Inizializzazione dei parametri dell'oggetto
        ros::NodeHandle nh;
        /* Service per la cancellazione della mappa Octomap
        clear_octomap_srv_ = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
        clear_octomap_srv_.waitForExistence();
        ROS_INFO("Connected to clear_octomap service.");*/

        // Server di azioni per pick e place
        pick_as_ = new actionlib::SimpleActionServer<tiago_pick_and_place::PickUpPoseAction>(nh, "pick_pose", boost::bind(&PickAndPlaceServer::pick_cb, this, _1), false);
        pick_as_->start();
        place_as_ = new actionlib::SimpleActionServer<tiago_pick_and_place::PickUpPoseAction>(nh, "place_pose", boost::bind(&PickAndPlaceServer::place_cb, this, _1), false);
        place_as_->start();
    }

    // Callback per il pick
    void pick_cb(const tiago_pick_and_place::PickUpPoseGoalConstPtr &goal) {
        int error_code = grasp_object(move_group, goal->object_pose,goal->object_name,goal->surface);
        tiago_pick_and_place::PickUpPoseResult result;
        result.error_code = error_code;
        if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            pick_as_->setAborted(result);
        } else {
            pick_as_->setSucceeded(result);
        }
    }

    // Callback per il place
    void place_cb(const tiago_pick_and_place::PickUpPoseGoalConstPtr &goal) {
        int error_code = place_object(move_group, goal->object_pose,goal->object_name,goal->surface);
        tiago_pick_and_place::PickUpPoseResult result;
        result.error_code = error_code;
        if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            place_as_->setAborted(result);
        } else {
            place_as_->setSucceeded(result);
        }
    }

    void openGripper(trajectory_msgs::JointTrajectory& posture){
        ROS_INFO("DEBUG-------------FUNZIONE OPEN GRIPPER");
        /* Add both finger joints of panda robot. */
        posture.joint_names.resize(2);
        posture.joint_names[0] = "gripper_left_finger_joint";
        posture.joint_names[1] = "gripper_right_finger_joint";

        /* Set them as open, wide enough for the object to fit. */
        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.04;
        posture.points[0].positions[1] = 0.04;
        posture.points[0].time_from_start = ros::Duration(2);
    }

    void closedGripper(trajectory_msgs::JointTrajectory& posture){
        ROS_INFO("DEBUG-------------FUNZIONE CLOSED GRIPPER");
        /* Add both finger joints of . */
        posture.joint_names.resize(2);
        posture.joint_names[0] = "gripper_left_finger_joint";
        posture.joint_names[1] = "gripper_right_finger_joint";

        /* Set them as closed. */
        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.022;
        posture.points[0].positions[1] = 0.022;
        posture.points[0].time_from_start = ros::Duration(1);
        
    }

    int grasp_object(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped &object_pose, const std::string &object_name,const std::string &surface) {
        ROS_INFO("DEBUG-------------INIZIO SEND PICKGOAL");
        /*ROS_INFO("Clearing octomap...");
        std_srvs::Empty srv;
        clear_octomap_srv_.call(srv);*/
        moveit_msgs::PickupGoal goal;
        goal.target_name = object_name;
        goal.group_name = "arm_torso";

        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        grasps[0].grasp_pose.header.frame_id = object_pose.header.frame_id;
        grasps[0].grasp_pose.pose = object_pose.pose;

        grasps[0].pre_grasp_approach.direction.header.frame_id = "arm_tool_link";
        grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.12;
        grasps[0].pre_grasp_approach.desired_distance = 0.2;

        grasps[0].post_grasp_retreat.direction.header.frame_id = "arm_tool_link";
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.12;
        grasps[0].post_grasp_retreat.desired_distance = 0.2;

        openGripper(grasps[0].pre_grasp_posture);
        closedGripper(grasps[0].grasp_posture);

        goal.possible_grasps = grasps;
        goal.support_surface_name = surface;
        goal.allow_gripper_support_collision = false;
        goal.attached_object_touch_links = {"gripper_left_finger_link", "gripper_right_finger_link", "gripper_link","wrist_ft_tool_link","wrist_ft_link"};
        goal.allowed_planning_time = 35.0;
        goal.planning_options.planning_scene_diff.is_diff = true;
        goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
        goal.planning_options.plan_only = false;
        goal.planning_options.replan = true;
        goal.planning_options.replan_attempts = 5;
        goal.allowed_touch_objects = {};
        ROS_INFO("DEBUG-------------MANDO LA MOVE_GROUP.PICK");

        move_group.setStartStateToCurrentState();
        moveit::core::MoveItErrorCode temp= move_group.pick(goal);
        ROS_INFO("DEBUG------------- RETURNO IL VAL");
        return temp.val;
    }

    int place_object(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped &object_pose, const std::string &object_name, const std::string &surface) {
        ROS_INFO("DEBUG-------------INIZIO SEND PLACE GOAL");
        /*ROS_INFO("Clearing octomap...");
        std_srvs::Empty srv;
        clear_octomap_srv_.call(srv);*/
        moveit_msgs::PlaceGoal goal;
        goal.attached_object_name = object_name;
        goal.group_name = "arm_torso";

        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);
        place_location[0].place_pose.header.frame_id = object_pose.header.frame_id;
        place_location[0].place_pose.pose = object_pose.pose;

        place_location[0].pre_place_approach.direction.header.frame_id = "arm_tool_link";
        place_location[0].pre_place_approach.direction.vector.z = 1.0;
        place_location[0].pre_place_approach.min_distance = 0.12;
        place_location[0].pre_place_approach.desired_distance = 0.2;

        place_location[0].post_place_retreat.direction.header.frame_id = "arm_tool_link";
        place_location[0].post_place_retreat.direction.vector.x = -1.0;
        place_location[0].post_place_retreat.min_distance = 0.12;
        place_location[0].post_place_retreat.desired_distance = 0.25;

        openGripper(place_location[0].post_place_posture);

        goal.place_locations = place_location;
        goal.place_eef = true;
        goal.support_surface_name = surface;
        goal.allow_gripper_support_collision = false;
        goal.allowed_touch_objects = {"gripper_left_finger_link", "gripper_right_finger_link", "gripper_link","wrist_ft_tool_link","wrist_ft_link"};
        goal.allowed_planning_time = 15.0;
        goal.planning_options.planning_scene_diff.is_diff = true;
        goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
        goal.planning_options.plan_only = false;
        goal.planning_options.replan = true;
        goal.planning_options.replan_attempts = 5;
        ROS_INFO("DEBUG-------------MANDO MOVE GROUP PLACE");
        moveit::core::MoveItErrorCode temp= move_group.place(goal);
        ROS_INFO("DEBUG------------- RETURNO IL VAL");
        return temp.val;

    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    actionlib::SimpleActionServer<tiago_pick_and_place::PickUpPoseAction>* pick_as_;
    actionlib::SimpleActionServer<tiago_pick_and_place::PickUpPoseAction>* place_as_;
    //ros::ServiceClient clear_octomap_srv_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_server");
    PickAndPlaceServer paps;
    ros::spin();
    return 0;
}