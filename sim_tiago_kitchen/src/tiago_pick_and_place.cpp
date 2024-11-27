#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//service provided
#include <sim_tiago_kitchen/PickPlaceCall.h>
// actions used
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

//moveit
#include <moveit/move_group_interface/move_group_interface.h>

std::vector<geometry_msgs::Pose> pick_pringles;
std::vector<geometry_msgs::Pose> pick_mug;
std::vector<geometry_msgs::Pose> pick_sprite;
std::vector<geometry_msgs::Pose> place_pringles;
std::vector<geometry_msgs::Pose> place_mug;
std::vector<geometry_msgs::Pose> place_sprite;

ros::Publisher twist_pub ;
ros::ServiceClient close_gripper_srv;
ros::ServiceClient open_gripper_srv;



geometry_msgs::Pose taskToPose(float x,float y,float z,float Rx,float Ry,float Rz){ //conversione da task di reuleaux a geometry_msgs::poses
        geometry_msgs::Pose point;
        point.position.x=x;
        point.position.y=y;
        point.position.z=z;
        tf2::Quaternion quat;
        quat.setRPY(Rx*3.14/180,Ry*3.14/180,Rz*3.14/180);
        point.orientation=tf2::toMsg(quat);
        return point;
}

void store_tasks(){ //salvo le task negli array globali, un oggetto per volta, copiando incollando le task dai .yaml di reuleaux (si è brutto ma efficace)
    pick_pringles.push_back(taskToPose(0.8129568099975586, -0.9642423391342163, 1.117740154266357, -90.81113195667926, 47.96383643831138, -92.96487432739953));
    pick_pringles.push_back(taskToPose(0.7361922860145569, -1.009443998336792, 1.059582829475403, 91.74079828492033, 52.0486021339228, -59.39690853044742));
    pick_pringles.push_back(taskToPose(0.8818308115005493, -0.9185670018196106, 0.8914331793785095, -89.98609685467324, -9.64607889196644, -109.33492761268));
    pick_pringles.push_back(taskToPose(0.9530173540115356, -1.009952902793884, 1.015895247459412, -90.16130400107706, 36.68970817033399, -140.6712008693245));
    place_pringles.push_back(taskToPose(1.08, 1.21, 1.29, 100.154, 29.603, 102.262));

    pick_mug.push_back(taskToPose(0.93, -1.23337733745575, 1.104748129844666, 65.47676703847849, 77.97409489539824, -25.87356871191176));
    pick_mug.push_back(taskToPose(0.9712081551551819, -1.065636277198792, 1.031280279159546, 89.11282208549174, 42.88137758318867, -90.23342102152429));
    pick_mug.push_back(taskToPose(0.982599675655365, -1.248825669288635, 1.118794441223145, -179.9999374146811, 89.71621217147975, -179.999948952354));
    pick_mug.push_back(taskToPose(0.9774245023727417, -1.168664693832397, 1.105125427246094, 0.7884831018062345, 70.26118166307035, -91.06168318787047));
    place_mug.push_back(taskToPose(0.01142683438956738, 1.198228001594543, 1.242118000984192, 86.35661748444021, 36.71599042359821, 94.65331964551396));

    pick_sprite.push_back(taskToPose(0.665, -1.203, 1.179, 88.477, 76.522, -48.339));
    pick_sprite.push_back(taskToPose(0.714, -1.244, 1.173, 179.999, 87.181, 179.999));
    pick_sprite.push_back(taskToPose(0.706, -1.0350, 0.998, -93.018, 20.973, -91.081));
    pick_sprite.push_back(taskToPose(0.596, -1.087, 0.983, -94.204, 21.462, -55.717));
    place_sprite.push_back(taskToPose(0.9824782609939575, -1.118773221969604, 1.153997421264648, -16.40400594651022, 62.92900499132745, -72.43401083339511));   
}

void move_base(bool back){
    int loop;
    float vel;
    if (back){
        loop=5;
        vel=-0.6;
    }else{
        loop=3;
        vel=0.3;
    }
    for(int i=0;i<=loop;i++){
        geometry_msgs::Twist msg; msg.linear.x = vel;
        twist_pub.publish(msg);
        ros::Duration(0.3).sleep();
    }
}

bool do_motion(std::string motion_name){
    actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> motion_client("/play_motion", true);
    if(!motion_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("play_motion action server not available");
        return false;
    }

    play_motion_msgs::PlayMotionGoal goal;
    goal.motion_name = motion_name;
    goal.skip_planning = false;
    goal.priority = 0;

    ROS_INFO_STREAM("Sending play_motion goal");
    motion_client.sendGoal(goal);

    ROS_INFO("Waiting for result ...");
    bool actionOk = motion_client.waitForResult(ros::Duration(30.0));

    actionlib::SimpleClientGoalState state = motion_client.getState();

    if ( actionOk ){
        ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
        return true;
    }else{
        ROS_ERROR_STREAM("Action failed with state: " << state.toString());
        return false;
    }
}

bool move_gripper(bool close){
    std_srvs::Empty srv;
    bool result;
    if(close){
        result=close_gripper_srv.call(srv);
    }else{
        result=open_gripper_srv.call(srv);
    }

    if (result) {
        ROS_INFO("gripper service call succeeded");
        return true;
    } else {
        ROS_ERROR("gripper service call failed");
        return false;
    }

}

geometry_msgs::Pose move_to_task(std::vector<geometry_msgs::Pose> task_poses,bool pick){
    geometry_msgs::Pose ret_pose;
    bool success;
    moveit::planning_interface::MoveGroupInterface group_arm("arm");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    for (int i=0; i<task_poses.size();i++){
        ROS_INFO("attempting plan task %d of %d",i+1,task_poses.size());
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.pose=task_poses[i];
        if (pick){
            goal_pose.pose.position.z+=0.05;
            ROS_DEBUG("DEBUG------ +0.05");
        }
        //choose your preferred planner
        group_arm.setPlannerId("SBLkConfigDefault");
        group_arm.setPoseReferenceFrame("map");
        group_arm.setPoseTarget(goal_pose);
        group_arm.setStartStateToCurrentState();
        group_arm.setMaxVelocityScalingFactor(1.0);

        
        //set maximum time to find a plan
        group_arm.setPlanningTime(10.0);
        success = bool(group_arm.plan(my_plan));

        if ( !success ){
            ROS_WARN("No plan found - attempting next pose");
        }else{
            ret_pose= task_poses[i];
            break;
        }
    }

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    // Execute plan
    ROS_INFO("DEBUG---- move to pose above task");
    ros::Time start = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode e = group_arm.move();
    if (!bool(e)){
        ROS_ERROR("Error executing plan");
    }
    if(pick){
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.pose=ret_pose;
        //choose your preferred planner
        group_arm.setPlannerId("SBLkConfigDefault");
        group_arm.setPoseReferenceFrame("map");
        group_arm.setPoseTarget(goal_pose);
        group_arm.setStartStateToCurrentState();
        group_arm.setMaxVelocityScalingFactor(1.0);
        //set maximum time to find a plan
        group_arm.setPlanningTime(10.0);
        success = bool(group_arm.plan(my_plan));
        if(success){
            ROS_INFO("debug --- plan per raggiungere task trovato - eseguo");
            ros::Time start = ros::Time::now();
            moveit::planning_interface::MoveItErrorCode e = group_arm.move();
            if (!bool(e)){
                ROS_ERROR("Error executing plan");
            }
        }else{
            ROS_ERROR("failed to reach task");
        }
    }

    return ret_pose;
}

bool retreat_from_task(geometry_msgs::Pose pose){
    ROS_INFO("DEBUG---- PLANNING POSE ABOVE TASK");
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose=pose;
    goal_pose.pose.position.z+=0.1;

    moveit::planning_interface::MoveGroupInterface group_arm("arm");
    //choose your preferred planner
    group_arm.setPlannerId("SBLkConfigDefault");
    group_arm.setPoseReferenceFrame("map");
    group_arm.setPoseTarget(goal_pose);
    group_arm.setStartStateToCurrentState();
    group_arm.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //set maximum time to find a plan
    group_arm.setPlanningTime(10.0);
    bool success = bool(group_arm.plan(my_plan));

    if ( !success ){
        ROS_WARN("No plan found");
    }

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    // Execute the plan
    ROS_INFO("DEBUG----- EXECUTING PLAN");
    ros::Time start = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode e = group_arm.move();
    if (!bool(e)){
        ROS_ERROR("Error executing plan");
        return false;
    }
    
    return true;
}

// Funzione per inviare il goal e pubblicare Twist
int execute_task(std::vector<geometry_msgs::Pose> tasks, bool pick, bool do_pregrasp){
    if(do_pregrasp){
        move_base(true); //move back - away from table
        if (!do_motion("pregrasp")){
            return -2; //error_code=INVALID_MOTION_PLAN
        }
        move_base(false); //move forward (back to the optimal bp) 
    }
    if(pick){
        ROS_INFO("opening gripper");
        move_gripper(false); //open gripper
    }
    geometry_msgs::Pose empty_pose;
    geometry_msgs::Pose task_done=move_to_task(tasks,pick);
    if(task_done==empty_pose){
        return 99999; //error_code=FAILURE
    }
   
    ROS_INFO("closing gripper");
    move_gripper(pick); //pick=true=close -- pick=false=open
    //ROS_INFO("attempting retreat");
    //retreat_from_task(task_done);
    if(do_pregrasp){
        move_base(true); //move back - away from table
        if (!do_motion("hold_object_home")){
            return -2; //error_code=INVALID_MOTION_PLAN
        }
    }
    
    return 1; //all done ok - error_code=SUCCESS
}

// Callback del servizio pick
bool pick_cb(sim_tiago_kitchen::PickPlaceCall::Request &req, sim_tiago_kitchen::PickPlaceCall::Response &res){   
    ROS_INFO("DEBUG---- object to pick: %s", req.task_name.c_str());
    std::vector<geometry_msgs::Pose> tasks;
    if(req.task_name=="pringles"){
        tasks=pick_pringles;
    }else if(req.task_name=="mug"){
        tasks=pick_mug;
    }else if(req.task_name=="sprite"){
        tasks=pick_sprite;
    }
    // Rispondi al servizio
    res.error_code = execute_task(tasks,true,req.do_pregrasp);
    return true;
}

// Callback del servizio place
bool place_cb(sim_tiago_kitchen::PickPlaceCall::Request &req, sim_tiago_kitchen::PickPlaceCall::Response &res){
    ROS_INFO("DEBUG---- object to place: %s", req.task_name.c_str());
    std::vector<geometry_msgs::Pose> tasks;
    if(req.task_name=="pringles"){
        tasks=place_pringles;
    }else if(req.task_name=="mug"){
        tasks=place_mug;
    }else if(req.task_name=="sprite"){
        tasks=place_sprite;
    }
    // Rispondi al servizio
    res.error_code = execute_task(tasks,false,req.do_pregrasp);
    return true;
}

int main(int argc, char** argv){
    // Inizializza il nodo ROS
    ros::init(argc, argv, "tiago_pick_and_place");
    ros::NodeHandle nh;
        ros::AsyncSpinner spinner(2);
        spinner.start();
    store_tasks();
    // Publisher per inviare messaggi Twist
    twist_pub = nh.advertise<geometry_msgs::Twist>("/key_vel", 10);
    //server client for gripper control
    open_gripper_srv=nh.serviceClient<std_srvs::Empty>("parallel_gripper_controller/release");
    close_gripper_srv=nh.serviceClient<std_srvs::Empty>("parallel_gripper_controller/grasp");

    // Server per il servizio PickPlaceCall
    ros::ServiceServer pick_srv = nh.advertiseService("tiago_pick_and_place/pick", pick_cb);
    ros::ServiceServer place_srv = nh.advertiseService("tiago_pick_and_place/place", place_cb);
    ROS_INFO("PICK AND PLACE NODE READY");

    //ros::spin();
    ros::waitForShutdown();
    return 0;
}