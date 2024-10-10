// ROS
#include <ros/ros.h>
#include <XmlRpcValue.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupGoal.h>
#include <moveit_msgs/PlaceGoal.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Services
#include <tiago_pick_and_place/PickUpName.h>
#include <tiago_pick_and_place/MoveAll.h>

// Setup
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

// Classe per gestire il pick and place
class PickPlaceManager {
public:
PickPlaceManager() :move_group("arm_torso") {
        ros::NodeHandle nh;
        // Creazione client necessario
        gazebo_model_cl= nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        // Setup iniziale
        loadTiagoPoses();
        loadObjectInfo("/pick_and_place_handler/pick_task", true);
        addCollisionObjects();
        // Pubblicazione servizi
        move_all_srv= nh.advertiseService("move_obj_on_tiago", &PickPlaceManager::move_all_cb, this);
        pick_name_srv= nh.advertiseService("pick_object", &PickPlaceManager::pick_name_cb, this);
        place_name_srv= nh.advertiseService("place_object", &PickPlaceManager::place_name_cb, this);
        ROS_INFO("DEBUG-------------FINE DEL CONSTRUCTOR");
    }

    geometry_msgs::Pose taskToPose( float x,float y,float z,float Rx,float Ry,float Rz){
        geometry_msgs::Pose point;
        point.position.x=x;
        point.position.y=y;
        point.position.z=z;
        tf2::Quaternion quat;
        quat.setRPY(Rx*3.14/180,Ry*3.14/180,Rz*3.14/180);
        point.orientation=tf2::toMsg(quat);
        return point;
    }
    
    // Funzione per caricare le pose di Tiago
    void loadTiagoPoses() {
        ROS_INFO("DEBUG----- Tiago poses");
        //pose per mettere gli oggetti sulla "schiena del tiago"
        //ANCHE QUESTO NON SAREBBE MALE FARLO DA YAML v- messi nella cartella config
        tiago_poses.resize(2);
        tiago_poses[0]=taskToPose(0, 0.085, 0.12, 1.237702952710297e-05, 15.08717871812678, -89.0701061595649);
        tiago_poses[1]=taskToPose(0, -0.085, 0.12, 179.9999887144125, 14.05228082906504, 90.92988488492941);

    }

    // Funzione per caricare le informazioni degli oggetti
    void loadObjectInfo(const std::string& file_ns, bool is_first_file) {
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
        object_names.resize(4);
        object_shapes.resize(4);  
        pick_poses.resize(4);
        place_poses.resize(4);


        object_names={"pringles","blue_mug","plate","ceramic_bowl"};

        
        object_shapes[0].type=object_shapes[0].CYLINDER;
        object_shapes[0].dimensions.resize(2);
        object_shapes[0].dimensions[object_shapes[0].CYLINDER_HEIGHT]=0.235;
        object_shapes[0].dimensions[object_shapes[0].CYLINDER_RADIUS]=0.04;
        object_shapes[1].type=object_shapes[1].CYLINDER;
        object_shapes[1].dimensions.resize(2);
        object_shapes[1].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.09;
        object_shapes[1].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.06;
        object_shapes[2].type=object_shapes[1].CYLINDER;
        object_shapes[2].dimensions.resize(2);
        object_shapes[2].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.03;
        object_shapes[2].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.098;
        object_shapes[3].type=object_shapes[1].CYLINDER;
        object_shapes[3].dimensions.resize(2);
        object_shapes[3].dimensions[object_shapes[1].CYLINDER_HEIGHT]=0.15;
        object_shapes[3].dimensions[object_shapes[1].CYLINDER_RADIUS]=0.155;

        //copy - paste the tasks from the .yaml files of base placement plugin
        pick_poses[0]=taskToPose(0.5906696319580078, -1.161354184150696, 1.008862972259521, 22.92856220074228, 26.54229966730917, 2.56273065787024);
        pick_poses[1]=taskToPose(0.5912778377532959, -1.239854574203491, 0.9866940975189209, 1.986290475503249e-06, 20.90574360665684, -97.05294404879315);
        pick_poses[2]=taskToPose(1.0917903542518616, -1.126552367210388, 0.8531627655029297, -1.93404715434553e-06, 11.37457259235728, -77.99777289082168);
        pick_poses[3]=taskToPose(0.9088554978370667, -1.254038214683533, 1.001855254173279, 91.16369232940949, 79.80283394894424, 14.2187638890949);


        place_poses[0]=taskToPose(-0.04285281151533127, 1.331524610519409, 1.158608436584473, -180.0000011550127, 63.34750887087267, 179.99999875803);
        place_poses[1]=taskToPose(0.4160979688167572, 1.29015851020813, 1.205958008766174, 34.39709229668563, 54.04448213358947, 10.41800142633139);
        place_poses[2]=taskToPose(-0.1331975758075714, 1.392423272132874, 1.136984944343567, 98.08313137781101, 70.42575173752006, 79.99466229173073);
        place_poses[3]=taskToPose(0.3122886419296265, 1.259033560752869, 1.195457816123962, 131.1369176361405, 63.86915928322281, 115.6369147159381);
        ROS_INFO("DEBUG----- OGGETTI FATTO");
    }

    // Funzione per ottenere la posa del modello da Gazebo
    geometry_msgs::Pose getModelPose(const std::string& model_name) {
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = model_name;
        geometry_msgs::Pose model_pose;
        if (gazebo_model_cl.call(srv)) {
            model_pose = srv.response.pose;
        } else {
            ROS_ERROR("Failed to call service get_model_state for model: %s", model_name.c_str());
        }
        ROS_INFO("DEBUG------GAZEBO pose ogg: %f,%f,%f,%f,%f,%f,%f",model_pose.position.x,model_pose.position.y,model_pose.position.z,model_pose.orientation.x,model_pose.orientation.y,model_pose.orientation.z,model_pose.orientation.w);
        return model_pose;
    }

    // Aggiunta oggetti di collisione
    void addCollisionObjects() {
        int obj_num = static_cast<int>(object_names.size());
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
            collision_objects[i].primitive_poses[0]=getModelPose(object_names[i]);
            collision_objects[i].primitive_poses[0].position.z=0.815+collision_objects[i].primitives[0].dimensions[0]/2;


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
    }

/////////////////////////////////////////////////// FUNCTIONS TO SEND PICK/PLACE GOALS TO MOVE_GROUP //////////////////////////////////////////////
   
    // Funzione per aprire il gripper
    void openGripper(trajectory_msgs::JointTrajectory& posture) {
    ROS_INFO("DEBUG-------------FUNZIONE OPEN GRIPPER");
        posture.joint_names.resize(2);
        posture.joint_names[0] = "gripper_left_finger_joint";
        posture.joint_names[1] = "gripper_right_finger_joint";

        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.042;
        posture.points[0].positions[1] = 0.042;
        posture.points[0].time_from_start = ros::Duration(2);
    }

    // Funzione per chiudere il gripper
    void closedGripper(trajectory_msgs::JointTrajectory& posture) {
    ROS_INFO("DEBUG-------------FUNZIONE CLOSED GRIPPER");
        posture.joint_names.resize(2);
        posture.joint_names[0] = "gripper_left_finger_joint";
        posture.joint_names[1] = "gripper_right_finger_joint";

        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.02;
        posture.points[0].positions[1] = 0.02;
        posture.points[0].time_from_start = ros::Duration(1);
    }

    // Funzione per inviare il goal di pick
    int sendPickGoal(const std::string& name, const geometry_msgs::Pose& pose, const std::string& frame, const std::string& surface) {
        ROS_INFO("DEBUG-------------INIZIO SEND PICKGOAL");
        moveit_msgs::PickupGoal goal;
        goal.target_name = name;
        goal.group_name = "arm_torso";

        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        grasps[0].grasp_pose.header.frame_id = frame;
        grasps[0].grasp_pose.pose = pose;

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
    // Funzione per inviare il goal di place
    int sendPlaceGoal(const std::string& name, const geometry_msgs::Pose& pose, const std::string& frame, const std::string& surface) {
        ROS_INFO("DEBUG-------------INIZIO SEND PLACE GOAL");
        moveit_msgs::PlaceGoal goal;
        goal.attached_object_name = name;
        goal.group_name = "arm_torso";

        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);
        place_location[0].place_pose.header.frame_id = frame;
        place_location[0].place_pose.pose = pose;

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
///////////////////////////////////////////////////// FUNCTIONS FOR SERVICES pick_object, place_object, move_all :  ////////////////////////////////////////////////////////////////////////////
    int pick_place_name(std::string name, bool tiago, bool pick){
        ROS_INFO("DEBUG-------------INIZIO PICKPLACE NAME");
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
                ROS_INFO("DEBUG-------------PICK TIAGO");
                ret=sendPickGoal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back");
            }else{
                ROS_INFO("DEBUG-------------PICK TABLE");
                ret=sendPickGoal(object_names[i],pick_poses[i],"map","table_pick");
            }
        }else{
            if(tiago){
                ROS_INFO("DEBUG-------------PLACE TIAGO");
                ret=sendPlaceGoal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back");
            }else{
                ROS_INFO("DEBUG-------------PLACE TABLE");
                ret=sendPlaceGoal(object_names[i],place_poses[i],"map","table_place");
            }
        }
        return ret;
    }

    int move_objects(bool pick){
        ROS_INFO("DEBUG-------------INIZIO MOVE OBJECTS DI MOVE ALL");
        if (pick){ //prendo dal tavolo e posiziono su tiago per il trasporto
            for (int i = 0; i < tiago_poses.size(); i++){
                ret=sendPickGoal(object_names[i],pick_poses[i],"map","table_pick");
                if(ret!=1){ //if not SUCCESS
                    return ret;
                }
                ret=sendPlaceGoal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back");
                if(ret!=1){ //if not SUCCESS
                    return ret;
                }
            }
        }else{ //prendo dal tiago e metto a posto
            for (int i = 0; i < tiago_poses.size(); i++){
                ret=sendPickGoal(object_names[i],tiago_poses[i],"torso_lift_link","tiago_back");
                if(ret!=1){ //if not SUCCESS
                    return ret;
                }
                ret=sendPlaceGoal(object_names[i],place_poses[i],"map","table_place");
                if(ret!=1){ //if not SUCCESS
                    return ret;
                }
            }
        }
        return moveit_msgs::MoveItErrorCodes::SUCCESS;
    }

    // Callback per il pick_object
    bool pick_name_cb(tiago_pick_and_place::PickUpName::Request &req, tiago_pick_and_place::PickUpName::Response &res) {
        ROS_INFO("DEBUG-------------INIZIO PICK NAME CB");
        int error_code = pick_place_name(req.object_name,req.tiago,true);
        ROS_INFO("DEBUG-------------PICK NAME CB - SALVO ERROR CODE");
        res.error_code = error_code;
        ROS_INFO("DEBUG-------------PICK NAME CB - RETURNO");
        return true;
    }

    // Callback per il pick_object
    bool place_name_cb(tiago_pick_and_place::PickUpName::Request &req, tiago_pick_and_place::PickUpName::Response &res) {
        ROS_INFO("DEBUG-------------INIZIO PLACE NAME CB");
        int error_code = pick_place_name(req.object_name,req.tiago,false);
        ROS_INFO("DEBUG-------------PLACE NAME CB - SALVO ERROR CODE");
        res.error_code = error_code;
        ROS_INFO("DEBUG------------ PLACE NAME CB - RETURNO");
        return true;
    }

    // Callback per il move_all
    bool move_all_cb(tiago_pick_and_place::MoveAll::Request &req, tiago_pick_and_place::MoveAll::Response &res) {
        int error_code = move_objects(req.pick);
        res.error_code = error_code;
        return true;
    }

private:
    // Variabili membro
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::ServiceClient gazebo_model_cl;

    std::vector<std::string> object_names;
    std::vector<shape_msgs::SolidPrimitive> object_shapes;
    std::vector<geometry_msgs::Pose> pick_poses;
    std::vector<geometry_msgs::Pose> place_poses;
    std::vector<geometry_msgs::Pose> tiago_poses;

    ros::ServiceServer move_all_srv;
    ros::ServiceServer pick_name_srv;
    ros::ServiceServer place_name_srv;

    int ret;
};

// Funzione main
int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_and_place_server_full");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("DEBUG-------------INIZIO TUTTO");
    PickPlaceManager manager;
    ros::waitForShutdown();
    return 0;
}