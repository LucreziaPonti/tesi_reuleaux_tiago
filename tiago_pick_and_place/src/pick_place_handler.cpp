
// ROS
#include <ros/ros.h>
#include <XmlRpcValue.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/client/simple_action_client.h>
#include <tiago_pick_and_place/PickUpPoseAction.h>

#include <tiago_pick_and_place/PickUpName.h>
#include <tiago_pick_and_place/MoveAll.h>

#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

ros::ServiceClient gazebo_model_cl;
std::vector<geometry_msgs::Pose> tiago_poses;
std::vector<std::string> object_names;
std::vector<shape_msgs::SolidPrimitive> object_shapes;
std::vector<geometry_msgs::Pose> pick_poses;
std::vector<geometry_msgs::Pose> place_poses;
actionlib::SimpleActionClient<tiago_pick_and_place::PickUpPoseAction>* pick_ac;
actionlib::SimpleActionClient<tiago_pick_and_place::PickUpPoseAction>* place_ac;
ros::ServiceServer move_all_srv;
ros::ServiceServer pick_name_srv;
ros::ServiceServer place_name_srv;

void load_tiago_poses(ros::NodeHandle& nh){
  /*XmlRpc::XmlRpcValue object_list;
  ROS_INFO("DEBUG----inizio load obj info tiago"); 
  // Leggi la lista di oggetti dal namespace specificato
  nh.getParam("/pick_and_place_handler/tiago_task" , object_list);
  ROS_INFO("DEBUG----get param fatto"); 
  for (int i = 0; i < object_list.size(); ++i) {
    ROS_INFO("DEBUG---- for %d",i);
      std::string name = static_cast<std::string>(object_list[i]["name"]);
      ROS_INFO("DEBUG---- nome ok %d",i);
      std::vector<double> point;
      for (int j = 0; j < object_list[i]["point"].size(); ++j) {
          point.push_back(static_cast<double>(object_list[i]["point"][j]));
      }
      ROS_INFO("DEBUG---- preso nome e point"); 
      // Crea una geometry_msgs::Pose e riempi i dati
          geometry_msgs::Pose pose;
          pose.position.x = point[0];
          pose.position.y = point[1];
          pose.position.z = point[2];
          pose.orientation.x = point[3];
          pose.orientation.y = point[4];
          pose.orientation.z = point[5];
          pose.orientation.w = 1.0; // Default valore della w per una orientazione neutra

      // Aggiungi la pose all'array corrispondente
              object_names.push_back(name);
              pick_poses.push_back(pose);
          ROS_INFO("DEBUG----fatto push back"); 
      // Utilizza 'name' e 'point' per la tua logica
      ROS_INFO("Object: %s, Point: [%f, %f, %f, %f, %f, %f]",
                name.c_str(), point[0], point[1], point[2], point[3], point[4], point[5]);
  } */
  
  ROS_INFO("DEBUG----- Tiago poses");
    //pose per mettere gli oggetti sulla "schiena del tiago"
  //ANCHE QUESTO NON SAREBBE MALE FARLO DA YAML v- messi nella cartella config
  tiago_poses.resize(2);
  tiago_poses[0].position.x=0;
  tiago_poses[0].position.y=0.085;
  tiago_poses[0].position.z= 0.2;
  tiago_poses[0].orientation.x= 1.237702952710297e-05;
  tiago_poses[0].orientation.y= 15.08717871812678;
  tiago_poses[0].orientation.z= -89.070106159564;
  tiago_poses[0].orientation.w=1;
  tiago_poses[1].position.x= 0;
  tiago_poses[1].position.y=-0.085;
  tiago_poses[1].position.z= 0.2;
  tiago_poses[1].orientation.x= 179.9999887144125;
  tiago_poses[1].orientation.y= 14.05228082906504;
  tiago_poses[1].orientation.z= 90.92988488492941;
  tiago_poses[1].orientation.w=1;
  
}

void load_obj_info(ros::NodeHandle& nh,const std::string& file_ns, bool is_first_file) {
    /*XmlRpc::XmlRpcValue object_list;
    ROS_INFO("DEBUG----inizio load obj info %s",file_ns.c_str()); 
    // Leggi la lista di oggetti dal namespace specificato
    nh.getParam(file_ns , object_list);
ROS_INFO("DEBUG----get param fatto"); 
    for (int i = 0; i < object_list.size(); ++i) {
      ROS_INFO("DEBUG---- for %d",i);
        std::string name = static_cast<std::string>(object_list[i]["name"]);
        ROS_INFO("DEBUG---- nome ok %d",i);
        std::vector<double> point;
        for (int j = 0; j < object_list[i]["point"].size(); ++j) {
            point.push_back(static_cast<double>(object_list[i]["point"][j]));
        }
        ROS_INFO("DEBUG---- preso nome e point"); 
        // Crea una geometry_msgs::Pose e riempi i dati
            geometry_msgs::Pose pose;
            pose.position.x = point[0];
            pose.position.y = point[1];
            pose.position.z = point[2];
            pose.orientation.x = point[3];
            pose.orientation.y = point[4];
            pose.orientation.z = point[5];
            pose.orientation.w = 1.0; // Default valore della w per una orientazione neutra

        // Aggiungi la pose all'array corrispondente
            if (is_first_file) {
                // Aggiungi il nome all'array globale
                object_names.push_back(name);
                pick_poses.push_back(pose);
            } else {
                place_poses.push_back(pose);
            }
            ROS_INFO("DEBUG----fatto push back"); 
        // Utilizza 'name' e 'point' per la tua logica
        ROS_INFO("Object: %s, Point: [%f, %f, %f, %f, %f, %f]",
                 name.c_str(), point[0], point[1], point[2], point[3], point[4], point[5]);
    }
    ROS_INFO("DEBUG----- OGGETTI FATTO");
   */
  ROS_INFO("DEBUG-----------------CREO ARRAY CON OGGETTI E TASK");
  object_names.resize(2);
  pick_poses.resize(2);
  place_poses.resize(2);

  object_names={"pringles","plate"};

  pick_poses[0].position.x=0.8;
  pick_poses[0].position.y=-1.13294;
  pick_poses[0].position.z= 1.10987;
  pick_poses[0].orientation.x= 6.755080245423144e-06;
  pick_poses[0].orientation.y= 21.55470077604551;
  pick_poses[0].orientation.z=-78.41680282328241;
  pick_poses[0].orientation.w=1;
  pick_poses[1].position.x= 1.100000023841858;
  pick_poses[1].position.y=-1.370000004768372;
  pick_poses[1].position.z= 1.049999952316284;
  pick_poses[1].orientation.x= -2.581479198981087e-06;
  pick_poses[1].orientation.y= 11.37457230062308;
  pick_poses[1].orientation.z=-77.99777397015771;
  pick_poses[1].orientation.w=1;
  
  //array place poses 
  place_poses[0].position.x=-0.04285281151533127;
  place_poses[0].position.y=1.331524610519409;
  place_poses[0].position.z= 1.158608436584473;
  place_poses[0].orientation.x=  -180.0000011550127;
  place_poses[0].orientation.y=  63.34750887087267;
  place_poses[0].orientation.z= 179.99999875803;
  place_poses[0].orientation.w=1;
  place_poses[1].position.x= 0.4160979688167572;
  place_poses[1].position.y= 1.29015851020813;
  place_poses[1].position.z= 1.205958008766174;
  place_poses[1].orientation.x= 34.39709229668563;
  place_poses[1].orientation.y= 54.04448213358947;
  place_poses[1].orientation.z= 10.41800142633139;
  place_poses[1].orientation.w=1;
  ROS_INFO("DEBUG----- OGGETTI FATTO");

  // object shapes
  object_shapes.resize(2);
  object_shapes[0].type=object_shapes[0].CYLINDER;
  object_shapes[0].dimensions.resize(2);
  object_shapes[0].dimensions[object_shapes[0].CYLINDER_HEIGHT]=0.235;
  object_shapes[0].dimensions[object_shapes[0].CYLINDER_RADIUS]=0.04;
  object_shapes[1].type=object_shapes[1].CYLINDER;
  object_shapes[1].dimensions.resize(2);
  object_shapes[1].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.035;
  object_shapes[1].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.098;

}

// Funzione per ottenere le pose dei modelli da Gazebo
geometry_msgs::Pose getModelPose(const std::string& model_name) {
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name;
    geometry_msgs::Pose model_pose;
    if (gazebo_model_cl.call(srv)) {
        model_pose = srv.response.pose;
    } else {
        ROS_ERROR("Failed to call service get_model_state for model: %s", model_name.c_str());
    }
    ROS_INFO("DEBUG------pose ogg: %f,%f,%f,%f,%f,%f,%f",model_pose.position.x,model_pose.position.y,model_pose.position.z,model_pose.orientation.x,model_pose.orientation.y,model_pose.orientation.z,model_pose.orientation.w);
    return model_pose;
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, const std::vector<std::string> object_names)
{
  int obj_num = static_cast<int>(object_names.size());
ROS_INFO("DEBUG------ INIZIO CREAZIONE COLLISION OBJ");
  // Creating Environment
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(obj_num+3);

  for(int i = 0; i < obj_num; ++i){
    // Define the object that we will be manipulating
    collision_objects[i].header.frame_id = "odom";
    collision_objects[i].id = object_names[i];

    ROS_INFO("DEBUG------OGGETTO %d ",i);
    /* Define the primitive and its dimensions. */
    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
    collision_objects[i].primitives[0].dimensions.resize(3);
    //DA CAPIRE COME SISTEMARE -- DIMENSIONI PROVVISORIE PER TESTARE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
    collision_objects[i].primitives[0].dimensions[0] = 0.1;
    collision_objects[i].primitives[0].dimensions[1] = 0.1;
    collision_objects[i].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0]=getModelPose(object_names[i]);
    collision_objects[i].primitive_poses[0].position.z=0.945+0.1;
    
    // END_SUB_TUTORIAL

    collision_objects[i].operation = collision_objects[i].ADD;
    ROS_INFO("DEBUG------AGGIUNTO");
  }
  ROS_INFO("DEBUG------INIZIO A METTER TAVOLO PICK");
  // Add the first table where the objects will originally be kept.
  collision_objects[obj_num].id = "table_pick"; //tavolo
  collision_objects[obj_num].header.frame_id = "odom";

  /* Define the primitive and its dimensions. */
  collision_objects[obj_num].primitives.resize(1);
  collision_objects[obj_num].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[obj_num].primitives[0].dimensions.resize(3);
  collision_objects[obj_num].primitives[0].dimensions[0] = 0.77;
  collision_objects[obj_num].primitives[0].dimensions[1] = 0.7;
  collision_objects[obj_num].primitives[0].dimensions[2] = 0.94;

  /* Define the pose of the table. */
  collision_objects[obj_num].primitive_poses.resize(1);
  collision_objects[obj_num].primitive_poses[0].position.x = 0.815;
  collision_objects[obj_num].primitive_poses[0].position.y = -1.35;
  collision_objects[obj_num].primitive_poses[0].position.z = 0.47;
  collision_objects[obj_num].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[obj_num].operation = collision_objects[obj_num].ADD;

  ROS_INFO("DEBUG------INIZIO A METTERE TAVOLO PLACE");
  // Add the second table where we will be placing the objects.
  collision_objects[obj_num+1].id = "table_place"; //lavabo
  collision_objects[obj_num+1].header.frame_id = "odom";

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
}

tiago_pick_and_place::PickUpPoseGoal crea_goal(std::string name, geometry_msgs::Pose pose, std::string frame, std::string surface){
  tiago_pick_and_place::PickUpPoseGoal goal;
  goal.object_name= name;
  goal.object_pose.pose= pose;
  goal.object_pose.header.frame_id= frame;
  goal.surface=surface;
  return goal;
}


int pick_place_name(std::string name, bool tiago, bool pick){
  //trova l'indice dell'oggetto
  int i;
  auto it = std::find(object_names.begin(), object_names.end(), name);
  if (it == object_names.end()){
    ROS_INFO("object not found");
    return moveit_msgs::MoveItErrorCodes::FAILURE;
  } else{
    i = std::distance(object_names.begin(), it);
    ROS_INFO("DEBUG---------- object index: %d",i);
    ROS_INFO("DEBUG---------- check obj info: %s - (%f,%f,%f)",object_names[i].c_str(),pick_poses[i].position.x,pick_poses[i].position.y,pick_poses[i].position.z);
  }

  if(pick){
    if(tiago){
      pick_ac->sendGoal(crea_goal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back"));  
    }else{
      pick_ac->sendGoal(crea_goal(object_names[i],pick_poses[i],"odom","table_pick"));
    }
    pick_ac->waitForResult();
    if(pick_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
      return moveit_msgs::MoveItErrorCodes::FAILURE;
    }
  }else{
    if(tiago){
      pick_ac->sendGoal(crea_goal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back"));  
    }else{
      place_ac->sendGoal(crea_goal(object_names[i],place_poses[i],"odom","table_place"));
    }
    place_ac->waitForResult();
    if(place_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
      return moveit_msgs::MoveItErrorCodes::FAILURE;
    }
  }
  return moveit_msgs::MoveItErrorCodes::SUCCESS;
}

int move_objects(bool pick){
  if (pick){ //prendo dal tavolo e posiziono su tiago per il trasporto
    for (int i = 0; i < object_names.size(); i++){
      pick_ac->sendGoal(crea_goal(object_names[i],pick_poses[i],"odom","table_pick"));
      pick_ac->waitForResult();
      if(pick_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
        return moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      place_ac->sendGoal(crea_goal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back"));
      place_ac->waitForResult();
      if(place_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
        return moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      
    }
  }else{ //prendo dal tiago e metto a posto
    for (int i = 0; i < object_names.size(); i++){
      pick_ac->sendGoal(crea_goal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back"));
      pick_ac->waitForResult();
      if(pick_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
        return moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      place_ac->sendGoal(crea_goal(object_names[i],place_poses[i],"odom","table_place"));
      place_ac->waitForResult();
      if(place_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
        return moveit_msgs::MoveItErrorCodes::FAILURE;
      }
      
    }
  }
  return moveit_msgs::MoveItErrorCodes::SUCCESS;
}

// Callback per il pick_by_name
bool pick_name_cb(tiago_pick_and_place::PickUpName::Request &req, tiago_pick_and_place::PickUpName::Response &res) {
    int error_code = pick_place_name(req.object_name,req.tiago,true);
    res.error_code = error_code;
    return true;
}


// Callback per il pick_by_name
bool place_name_cb(tiago_pick_and_place::PickUpName::Request &req, tiago_pick_and_place::PickUpName::Response &res) {
    int error_code = pick_place_name(req.object_name,req.tiago,false);
    res.error_code = error_code;
    return true;
}


// Callback per il move_all
bool move_all_cb(tiago_pick_and_place::MoveAll::Request &req, tiago_pick_and_place::MoveAll::Response &res) {
    int error_code = move_objects(req.pick);
    res.error_code = error_code;
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_handler");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //creiamo tutti i client che ci serviranno
  gazebo_model_cl = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); //per mettere i collision objects
  //per fare le azioni di pick and place
  pick_ac = new actionlib::SimpleActionClient<tiago_pick_and_place::PickUpPoseAction>("pick_pose", true);
  place_ac = new actionlib::SimpleActionClient<tiago_pick_and_place::PickUpPoseAction>("place_pose", true);

  ROS_INFO("Waiting for action servers...");
  pick_ac->waitForServer();
  place_ac->waitForServer();
  ROS_INFO("Action servers connected.");

  ros::WallDuration(1.0).sleep();
  load_tiago_poses(nh);
  //funzione che prende le info degli oggetti e del grasp
  //std::string pick_task_path;
    //nh.param("/pick_and_place_handler/pick_task_path", pick_task_path);
    //std::string place_task_path;
    //nh.param("/pick_and_place_handler/place_task_path", place_task_path);
    load_obj_info(nh,"/pick_and_place_handler/pick_task",true); //salva i nomi e le pose nell'array pick_pose
    //load_obj_info(nh,"place_task",false); //salva solo le pose nell'array place_pose

  //creiamo la planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionObjects(planning_scene_interface,object_names);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  
  //rendiamo disponibili i servizi per eseguire le diverse task
  move_all_srv = nh.advertiseService("move_all",move_all_cb);
  //new actionlib::SimpleActionServer<tiago_pick_and_place::MoveAllAction>(nh, "move_all", boost::bind(&move_all_cb, _1), false);
  pick_name_srv = nh.advertiseService("pick_by_name",pick_name_cb);
  //new actionlib::SimpleActionServer<tiago_pick_and_place::PickUpNameAction>(nh, "pick_by_name", boost::bind(&pick_name_cb, _1), false);
  place_name_srv = nh.advertiseService("place_by_name",place_name_cb);
  //new actionlib::SimpleActionServer<tiago_pick_and_place::PickUpNameAction>(nh, "place_by_name", boost::bind(&place_name_cb, _1), false);
  
  ros::waitForShutdown();
  return 0;
}

