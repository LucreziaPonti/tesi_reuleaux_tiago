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
    pick_pringles.push_back(taskToPose(0.5767598152160645, -1.161960124969482, 1.0158371925354, 22.92856404153541, 26.54230126434272, 2.562731003109279));
    pick_pringles.push_back(taskToPose(0.5767598152160645, -1.161960124969482, 1.0158371925354, 22.92856404153541, 26.54230126434272, 2.562731003109279));
    place_pringles.push_back(taskToPose(1.106438279151917, 1.347545027732849, 1.315836548805237, 68.910, 65.168, 68.134));
    place_pringles.push_back(taskToPose(1.079, 1.248, 1.369, 90, 0, 92.884));

    pick_mug.push_back(taskToPose(0.5767598152160645, -1.161960124969482, 1.0158371925354, 22.92856404153541, 26.54230126434272, 2.562731003109279));
    pick_mug.push_back(taskToPose(0.5767598152160645, -1.161960124969482, 1.0158371925354, 22.92856404153541, 26.54230126434272, 2.562731003109279));
    place_mug.push_back(taskToPose(-0.1394776403903961, 1.227737069129944, 1.238265037536621, 28.36199410416118, 67.65071302577509, 113.31308863419));

    pick_sprite.push_back(taskToPose(0.665, -1.203, 1.179, 88.477, 76.522, -48.339));
    pick_sprite.push_back(taskToPose(0.714, -1.244, 1.173, 179.999, 87.181, 179.999));
    pick_sprite.push_back(taskToPose(0.706, -1.0350, 0.998, -93.018, 20.973, -91.081));
    pick_sprite.push_back(taskToPose(0.596, -1.087, 0.983, -94.204, 21.462, -55.717));
    place_sprite.push_back(taskToPose(1.009, -1.117, 1.154, -16.404, 62.929, -72.434));   
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

geometry_msgs::Pose move_to_task(std::vector<geometry_msgs::Pose> task_poses){
    geometry_msgs::Pose ret_pose;
    bool success;
    moveit::planning_interface::MoveGroupInterface group_arm("arm");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    for (int i=0; i<task_poses.size();i++){
        ROS_INFO("attempting plan task %d of %d",i+1,task_poses.size());
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.pose=task_poses[i];
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
    // Execute the plan
    ros::Time start = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode e = group_arm.move();
    if (!bool(e)){
        ROS_ERROR("Error executing plan");
    }
    return ret_pose;
}

bool retreat_from_task(geometry_msgs::Pose pose){

    ROS_INFO("attempting plan retreat");
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose=pose;
    goal_pose.pose.position.z+=0.05;

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
        ROS_WARN("No plan found - attempting next pose");
    }

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    // Execute the plan
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
    geometry_msgs::Pose task_done=move_to_task(tasks);
    if(task_done==empty_pose){
        return 99999; //error_code=FAILURE
    }
   
    ROS_INFO("closing gripper");
    move_gripper(pick); //pick=true=close -- pick=false=open

    retreat_from_task(task_done);

    move_base(true); //move back - away from table

    if (!do_motion("hold_object_home")){
        return -2; //error_code=INVALID_MOTION_PLAN
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