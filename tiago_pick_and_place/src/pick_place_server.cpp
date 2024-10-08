#include <ros/ros.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Grasp.h>
//#include <moveit_msgs/PickupAction.h>
//#include <moveit_msgs/PlaceAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>
#include <tiago_pick_and_place/PickUpPoseAction.h>

// Mappa per gli errori di MoveIt
std::map<int, std::string> moveit_error_dict;
/*
void initializeMoveItErrorDict() {
    moveit_error_dict[moveit_msgs::MoveItErrorCodes::SUCCESS] = "SUCCESS";
    moveit_error_dict[moveit_msgs::MoveItErrorCodes::FAILURE] = "FAILURE";
    // Aggiungi altri errori secondo necessità
}

// Creazione del goal per l'azione di "pickup"
moveit_msgs::PickupGoal createPickupGoal(const std::string &group, const std::string &target, const geometry_msgs::PoseStamped &grasp_pose, const std::vector<moveit_msgs::Grasp> &possible_grasps, const std::vector<std::string> &links_to_allow_contact) {
    moveit_msgs::PickupGoal goal;
    goal.target_name = target;
    goal.group_name = group;
    goal.possible_grasps = possible_grasps;
    goal.allowed_planning_time = 35.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.plan_only = false;
    goal.planning_options.replan = true;
    goal.planning_options.replan_attempts = 1;
    goal.allowed_touch_objects = {};
    goal.attached_object_touch_links = links_to_allow_contact;

    return goal;
}

// Creazione del goal per l'azione di "place"
moveit_msgs::PlaceGoal createPlaceGoal(const geometry_msgs::PoseStamped &place_pose, const std::vector<moveit_msgs::PlaceLocation> &place_locations, const std::string &group, const std::string &target, const std::vector<std::string> &links_to_allow_contact) {
    moveit_msgs::PlaceGoal goal;
    goal.group_name = group;
    goal.attached_object_name = target;
    goal.place_locations = place_locations;
    goal.allowed_planning_time = 15.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.plan_only = false;
    goal.planning_options.replan = true;
    goal.planning_options.replan_attempts = 1;
    goal.allowed_touch_objects = links_to_allow_contact;

    return goal;
}
*/
class PickAndPlaceServer {
public:
    PickAndPlaceServer() 
    : move_group("arm")
    {
        ROS_INFO("Initializing PickAndPlaceServer...");


        /*// Client per le azioni di pick e place
        pickup_ac_ = new actionlib::SimpleActionClient<moveit_msgs::PickupAction>("/pickup", true);
        place_ac_ = new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>("/place", true);

        ROS_INFO("Waiting for action servers...");
        pickup_ac_->waitForServer();
        place_ac_->waitForServer();
        ROS_INFO("Action servers connected.");
*/
        // Inizializzazione dei parametri dell'oggetto
        ros::NodeHandle nh;
        //nh.param("links_to_allow_contact", links_to_allow_contact_, std::vector<std::string>());
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
        int error_code = grasp_object(move_group, goal->object_pose,goal->object_name);
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
        int error_code = place_object(move_group, goal->object_pose,goal->object_name);
        tiago_pick_and_place::PickUpPoseResult result;
        result.error_code = error_code;
        if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            place_as_->setAborted(result);
        } else {
            place_as_->setSucceeded(result);
        }
    }

    void openGripper(trajectory_msgs::JointTrajectory& posture)
    {
        // BEGIN_SUB_TUTORIAL open_gripper
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
        // END_SUB_TUTORIAL
    }

    void closedGripper(trajectory_msgs::JointTrajectory& posture)
    {
       
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

    int grasp_object(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped &object_pose, const std::string &object_name) {
        /*ROS_INFO("Clearing octomap...");
        std_srvs::Empty srv;
        clear_octomap_srv_.call(srv);*/
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        // Setting grasp pose
        grasps[0].grasp_pose.header.frame_id = object_pose.header.frame_id;
        grasps[0].grasp_pose.pose = object_pose.pose;
        // Setting pre-grasp approach
        /* Defined with respect to frame_id */
        grasps[0].pre_grasp_approach.direction.header.frame_id = "arm_tool_link";
        /* Direction is set as positive x axis */
        grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.12;
        grasps[0].pre_grasp_approach.desired_distance = 0.2;
        // Setting post-grasp retreat
        /* Defined with respect to frame_id */
        grasps[0].post_grasp_retreat.direction.header.frame_id = "arm_tool_link";
        /* Direction is set as positive z axis */
        grasps[0].post_grasp_retreat.direction.vector.y = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.12;
        grasps[0].post_grasp_retreat.desired_distance = 0.25;
        // Setting posture of eef before grasp
        openGripper(grasps[0].pre_grasp_posture);
        // Setting posture of eef during grasp
        closedGripper(grasps[0].grasp_posture);

        // Set support surface as table1.
        move_group.setSupportSurfaceName("table_pick");
        // Call pick to pick up the object using the grasps given
        move_group.pick(object_name, grasps);
        return moveit_msgs::MoveItErrorCodes::SUCCESS;
    }

    int place_object(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::PoseStamped &object_pose, const std::string &object_name) {
        /*ROS_INFO("Clearing octomap...");
        std_srvs::Empty srv;
        clear_octomap_srv_.call(srv);*/
        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);
        // Setting grasp pose
        place_location[0].place_pose.header.frame_id = object_pose.header.frame_id;
        place_location[0].place_pose.pose = object_pose.pose;
        
        // Setting pre-place approach
        /* Defined with respect to frame_id */
        place_location[0].pre_place_approach.direction.header.frame_id = "arm_tool_link";
        /* Direction is set as positive x axis */
        place_location[0].pre_place_approach.direction.vector.y = 1.0;
        place_location[0].pre_place_approach.min_distance = 0.12;
        place_location[0].pre_place_approach.desired_distance = 0.2;

        // Setting post-place retreat
        /* Defined with respect to frame_id */
        place_location[0].post_place_retreat.direction.header.frame_id = "arm_tool_link";
        /* Direction is set as positive y axis */
        place_location[0].post_place_retreat.direction.vector.y = -1.0;
        place_location[0].post_place_retreat.min_distance = 0.12;
        place_location[0].post_place_retreat.desired_distance = 0.25;

        // Setting posture of eef after placing
        openGripper(place_location[0].post_place_posture);

        // Set support surface as table1.
        move_group.setSupportSurfaceName("table_place");
        // Call pick to pick up the object using the place_location given
        move_group.place(object_name, place_location);
        

        return moveit_msgs::MoveItErrorCodes::SUCCESS;
    }

private:
    //moveit::planning_interface::PlanningSceneInterface* scene_;
    //actionlib::SimpleActionClient<moveit_msgs::PickupAction>* pickup_ac_;
    //actionlib::SimpleActionClient<moveit_msgs::PlaceAction>* place_ac_;
    moveit::planning_interface::MoveGroupInterface move_group;
    actionlib::SimpleActionServer<tiago_pick_and_place::PickUpPoseAction>* pick_as_;
    actionlib::SimpleActionServer<tiago_pick_and_place::PickUpPoseAction>* place_as_;
    
    //ros::ServiceClient clear_octomap_srv_;

    std::vector<std::string> links_to_allow_contact_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_server");
    PickAndPlaceServer paps;
    ros::spin();
    return 0;
}