/*
    function check_collision_objects:
        receives a pose
        creates a client for the get_planning_scene service that returns the enitre planning scene
        
        
    
*/

#include <base_placement_plugin/filter_collision_poses.h>

#include <moveit/planning_scene/planning_scene.h>


FilterCollisionPoses::FilterCollisionPoses(){};

bool FilterCollisionPoses::check_collision_objects( ros::NodeHandle& node, const float x, const float y, const float z, bool checkZ){
    ROS_INFO("DEBUG--------------------- INIZIO CHECK COLLISION OBJECTS for pose: %f, %f, %f", x,y,z);

    planning_scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    //srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    if (!planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene - assuming all poses valid");
        return true;
    }
    ROS_INFO("DEBUG--------------------- PLANNING SCENE OTTENUTA");
    std::vector<moveit_msgs::CollisionObject> objects = srv.response.scene.world.collision_objects;
    //ROS_INFO("DEBUG--------------------- ABBIAMO %d COLLISION OBJECTS", objects.size() );
    if (objects.size()==0){
        ROS_ERROR("No collision objects in the scene - assuming all poses valid");
        return true;
    }
    for(int i=0; i<objects.size();i++){
        //ROS_INFO("object %d - name %s", i, objects[i].id.c_str());
        //ROS_INFO("pos = %f , %f , %f ", objects[i].pose.position.x,objects[i].pose.position.y,objects[i].pose.position.z);
        //store the position values (shorter names for variables=less messy code)
        co_x=objects[i].pose.position.x;
        co_y=objects[i].pose.position.y;
        co_z=objects[i].pose.position.z;
        //ROS_INFO("primitive type: %d (box=1,cylinder=3)", objects[i].primitives[0].type);

        //store the values of dimensions depending on the primitive type - here we only used box and cylinder
        if(objects[i].primitives[0].type==1){ //BOX
            //ROS_INFO("dimensions: %f , %f , %f", objects[i].primitives[0].dimensions[0],objects[i].primitives[0].dimensions[1],objects[i].primitives[0].dimensions[2]); 
            dX=objects[i].primitives[0].dimensions[0]/2+0.15;
            dY=objects[i].primitives[0].dimensions[1]/2+0.15;
            dZ=objects[i].primitives[0].dimensions[2]/2+0.15;
            // the +0.15 is mainly for the tables 

        }else if (objects[i].primitives[0].type==3){ //CYLINDER
            //ROS_INFO("dimensions: %f , %f ", objects[i].primitives[0].dimensions[0],objects[i].primitives[0].dimensions[1]); 
            dX=objects[i].primitives[0].dimensions[1];
            dY=objects[i].primitives[0].dimensions[1];
            dZ=objects[i].primitives[0].dimensions[0]/2;
        }

        if((x>(co_x-dX))&&(x<(co_x+dX))&&(y>(co_y-dY))&&(y<(co_y+dY))){
            if(!checkZ){ //for poses already on the ground (VerticalRobotModel) - bc some coll_obj dont fully touch the ground but the real obj does
                //inside the bounds of the object -- not acceptable pose
                ROS_INFO("DEBUG---------- NOT acceptable pose - %s ",objects[i].id.c_str());
                return false;
            }else{ // for poses of the base of the manipulator (can be elevated) 
                if((z>(co_z-dZ))&&(z<(co_z+dZ))){
                    return false;
                }
            }
        }
        // if not in collision goes on to check next object
    }
    //not in collision with any object -- acceptable pose
    ROS_INFO("DEBUG---------- acceptable pose");
    return true;
}
