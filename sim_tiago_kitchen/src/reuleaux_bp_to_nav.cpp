/*
 waits for navigation to be available (wait for action move_base i think)

 receives the optimal bp (via service) from reuleaux bpp or from terminal 
 when it receives an empty goal it means that the goal recived RIGHT BEFORE is the BEST bp from reuleaux 
 so it stores it at the beginning of the array (index 0 - left empty from the start) and removes it from the end of the array
 (adds the orientation so that tiago is facing the table - not yet implemented)
 the service reuleaux_bp_to_nav/move_to_bp becomes available and no more bp can be received
 when the service is called it sends the goal to the navigation (starting with the first, if it fails it attempts the next in the array)

 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_srvs/Trigger.h"


bool receive;
bool service_avail;
std::vector<geometry_msgs::Pose> poses;
geometry_msgs::Pose empty_pose;

void bp_sub_cb(const geometry_msgs::Pose::ConstPtr& msg){
    geometry_msgs::Pose appo;
    appo.position=msg->position;
    appo.orientation=msg->orientation;
    if(receive){
        ROS_INFO(",pose received: [%f,%f,%f] [%f,%f,%f,%f]", appo.position.x, appo.position.y, appo.position.z, appo.orientation.x,appo.orientation.y, appo.orientation.z,appo.orientation.w);
        if(appo==empty_pose){
            poses[0]=poses[poses.size()-1];
            poses.pop_back();
            ROS_INFO("Saved best pose");
            receive=false;
            service_avail=true;
            ROS_INFO("DEBUG--- poses stored");
            for(int i=0;i<poses.size();i++){
                ROS_INFO("DEBUG--- pose %d : (%f,%f,%f) (%f,%f,%f,%f)",i,poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x,poses[i].orientation.y, poses[i].orientation.z,poses[i].orientation.w);
            }
        }else{
            poses.push_back(appo);
        }
        //FUNZIONE PER SISTEMARE ORIENTAMENTO? - per ora tengo che lo faccio a mano dopo
    }else{
        ROS_INFO("Can't receive new base placement to go to - call the /reuleaux_bp_to_nav/move_to_bp service to attempt move or reset_bp_to_nav to reset the node");
    }
}

bool send_goal_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if(service_avail){
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        for(int i=0;i<poses.size();i++){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose=poses[i];

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Moved to pose %d : (%f,%f,%f) (%f,%f,%f,%f)",i,poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x,poses[i].orientation.y, poses[i].orientation.z,poses[i].orientation.w);
                res.success=true;
                break;
            }else{
                if(i==poses.size()-1){
                    ROS_INFO("Couldn't reach any of the poses");
                    res.success=false;
                }
                ROS_INFO("Can't move to base pose %d : (%f,%f,%f) (%f,%f,%f,%f) - attempting next pose",i,poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x,poses[i].orientation.y, poses[i].orientation.z,poses[i].orientation.w);
            }
        }
        ROS_INFO("Resetting the poses array and the node to be used again");
        poses.clear();
        poses.push_back(empty_pose);
        receive=true;
        service_avail=false;
    }else{
        ROS_INFO("service not available at the moment - no poses stored");
        res.success=false;
    }
    return true;
}

bool reset_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    ROS_INFO("Resetting the node reuleaux_bp_to_nav:");
    service_avail=false;
    ROS_INFO("/reuleaux_bp_to_nav/move_to_bp service no longer available");
    ROS_INFO("DEBUG------- %d",poses.size());
    poses.clear();
    ROS_INFO("DEBUG------- %d",poses.size());
    poses.push_back(empty_pose);
    ROS_INFO("DEBUG------- %d",poses.size());
    receive=true;
    ROS_INFO("Base poses array emptied - ready to receive new poses");
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "reuleaux_bp_to_nav");
    ros::NodeHandle n; 
    
    poses.push_back(empty_pose); //store the first element as empty, it will be filled with the best pose later
    ROS_INFO("DEBUG------- %d",poses.size());
    ros::Subscriber bp_sub = n.subscribe("reuleaux_bp_to_nav/bp_poses", 1000, bp_sub_cb);
    receive=true;
    service_avail=false;
    ros::ServiceServer send_goal_srv = n.advertiseService("reuleaux_bp_to_nav/move_to_bp",send_goal_cb);
    ros::ServiceServer reset_srv = n.advertiseService("reuleaux_bp_to_nav/reset_bp_to_nav",reset_cb);

    ros::spin();

    return 0;
}