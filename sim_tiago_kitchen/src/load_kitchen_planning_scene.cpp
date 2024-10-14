
// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::Pose getModelPose( float x,float y,float z,float Rx,float Ry,float Rz){
        geometry_msgs::Pose point;
        point.position.x=x;
        point.position.y=y;
        point.position.z=z;
        tf2::Quaternion quat;
        quat.setRPY(Rx*3.14/180,Ry*3.14/180,Rz*3.14/180);
        point.orientation=tf2::toMsg(quat);
        return point;
    }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "load_kitchen_planning_scene");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  try{
    tfBuffer.lookupTransform("odom", "map", ros::Time::now(), ros::Duration(3.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform odom to map: %s", ex.what());
  }

  //LOAD OGGETTI
  int obj_num=4; 
  std::vector<std::string> object_names;
  std::vector<shape_msgs::SolidPrimitive> object_shapes;
  std::vector<std::vector<float>> object_poses;
  
  object_names.resize(obj_num);
  object_shapes.resize(obj_num);
  object_poses.resize(obj_num);
   
  object_names[0]="pringles";
  object_shapes[0].type=object_shapes[0].CYLINDER;
  object_shapes[0].dimensions.resize(2);
  object_shapes[0].dimensions[object_shapes[0].CYLINDER_HEIGHT]=0.235;
  object_shapes[0].dimensions[object_shapes[0].CYLINDER_RADIUS]=0.04;
  object_poses[0].resize(6);
  object_poses[0]={0.7, -1.13, 1, 0, 0, 2.62};
  
  object_names[1]="blue_mug";
  object_shapes[1].type=object_shapes[1].CYLINDER;
  object_shapes[1].dimensions.resize(2);
  object_shapes[1].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.09;
  object_shapes[1].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.06;
  object_poses[1].resize(6);
  object_poses[1]={0.9, -1.26, 1, 0, 0, 0};
  
  object_names[2]="plate";
  object_shapes[2].type=object_shapes[1].CYLINDER;
  object_shapes[2].dimensions.resize(2);
  object_shapes[2].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.03;
  object_shapes[2].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.098;
  object_poses[2].resize(6);
  object_poses[2]={1.1, -1.4, 1, 0, 0, 0};
  
  object_names[3]="cerwamic_bowl";
  object_shapes[3].type=object_shapes[1].CYLINDER;
  object_shapes[3].dimensions.resize(2);
  object_shapes[3].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.15;
  object_shapes[3].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.155;
  object_poses[3].resize(6);
  object_poses[3]={0.6, -1.44, 1, 0, 0, 0};
  
  
  //creiamo la planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  ROS_INFO("DEBUG------ INIZIO CREAZIONE COLLISION OBJ");
  // Creating Environment
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(obj_num+3);

  for(int i = 0; i < obj_num; ++i){
    // Define the object that we will be manipulating
    collision_objects[i].header.frame_id = "map";
    collision_objects[i].id = object_names[i];

    ROS_INFO("DEBUG------OGGETTO %d ",i);
    /* Define the primitive and its dimensions. */
    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0]= object_shapes[i];

    /* Define the pose of the object. */
    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0]=getModelPose(object_poses[i][0],object_poses[i][1],object_poses[i][2],object_poses[i][3],object_poses[i][4],object_poses[i][5]);
    collision_objects[i].primitive_poses[0].position.z=0.815+collision_objects[i].primitives[0].dimensions[0]/2;
    
    // END_SUB_TUTORIAL

    collision_objects[i].operation = collision_objects[i].ADD;
    ROS_INFO("DEBUG------AGGIUNTO");
  }
  ROS_INFO("DEBUG------INIZIO A METTER TAVOLO PICK");
  // Add the first table where the objects will originally be kept.
  collision_objects[obj_num].id = "table_pick"; //tavolo
  collision_objects[obj_num].header.frame_id = "map";

  /* Define the primitive and its dimensions. */
  collision_objects[obj_num].primitives.resize(1);
  collision_objects[obj_num].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[obj_num].primitives[0].dimensions.resize(3);
  collision_objects[obj_num].primitives[0].dimensions[0] = 0.82;
  collision_objects[obj_num].primitives[0].dimensions[1] = 0.7;
  collision_objects[obj_num].primitives[0].dimensions[2] = 0.81;

  /* Define the pose of the table. */
  collision_objects[obj_num].primitive_poses.resize(1);
  collision_objects[obj_num].primitive_poses[0].position.x = 0.81;
  collision_objects[obj_num].primitive_poses[0].position.y = -1.4;
  collision_objects[obj_num].primitive_poses[0].position.z = collision_objects[obj_num].primitives[0].dimensions[2]/2;
  collision_objects[obj_num].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[obj_num].operation = collision_objects[obj_num].ADD;

  ROS_INFO("DEBUG------INIZIO A METTERE TAVOLO PLACE");
  // Add the second table where we will be placing the objects.
  collision_objects[obj_num+1].id = "table_place"; //lavabo
  collision_objects[obj_num+1].header.frame_id = "map";

  /* Define the primitive and its dimensions. */
  collision_objects[obj_num+1].primitives.resize(1);
  collision_objects[obj_num+1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[obj_num+1].primitives[0].dimensions.resize(3);
  collision_objects[obj_num+1].primitives[0].dimensions[0] = 2.3;
  collision_objects[obj_num+1].primitives[0].dimensions[1] = 0.6;
  collision_objects[obj_num+1].primitives[0].dimensions[2] = 1;

  /* Define the pose of the table. */
  collision_objects[obj_num+1].primitive_poses.resize(1);
  collision_objects[obj_num+1].primitive_poses[0].position.x = 0.2;
  collision_objects[obj_num+1].primitive_poses[0].position.y = 1.5;
  collision_objects[obj_num+1].primitive_poses[0].position.z = 0.5;
  collision_objects[obj_num+1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[obj_num+1].operation = collision_objects[obj_num+1].ADD;
 
 ROS_INFO("DEBUG------INIZIO A METTERE RIPIANO TIAGO");
  // Add the second table where we will be placing the objects.
  collision_objects[obj_num+2].id = "tiago_back"; //tiago
  collision_objects[obj_num+2].header.frame_id = "torso_lift_link";

  // Define the primitive and its dimensions. 
  collision_objects[obj_num+2].primitives.resize(1);
  collision_objects[obj_num+2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[obj_num+2].primitives[0].dimensions.resize(3);
  collision_objects[obj_num+2].primitives[0].dimensions[0] = 0.265;
  collision_objects[obj_num+2].primitives[0].dimensions[1] = 0.325;
  collision_objects[obj_num+2].primitives[0].dimensions[2] = 0.01;

  // Define the pose of the table. 
  collision_objects[obj_num+2].primitive_poses.resize(1);
  collision_objects[obj_num+2].primitive_poses[0].position.x = -0.025;
  collision_objects[obj_num+2].primitive_poses[0].position.y = 0;
  collision_objects[obj_num+2].primitive_poses[0].position.z = 0;
  collision_objects[obj_num+2].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[obj_num+2].operation = collision_objects[obj_num+2].ADD;
  ROS_INFO("DEBUG------FATTO METTO NELLA INTERFACE");
  planning_scene_interface.applyCollisionObjects(collision_objects);

  //metto tiago_back come attached collision object
  ROS_INFO("DEBUG------attacco RIPIANO TIAGO");
  moveit_msgs::AttachedCollisionObject attached_cll_obj;
  attached_cll_obj.link_name="torso_lift_link";
  attached_cll_obj.object=collision_objects[obj_num+2];
  std::vector<std::string> touch_links; 
  touch_links.push_back("torso_lift_link");
  attached_cll_obj.touch_links=touch_links;
  planning_scene_interface.applyAttachedCollisionObject(attached_cll_obj);
  ROS_INFO("DEBUG------FATTO METTO NELLA INTERFACE");
 
  ros::waitForShutdown();
  return 0;
}

