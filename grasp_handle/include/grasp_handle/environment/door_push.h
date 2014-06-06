


#ifndef GRASP_HANDLE__DOOR_PUSH_
#define GRASP_HANDLE__DOOR_PUSH_
#include <moveit_visual_tools/visual_tools.h> // simple tool for showing grasps
namespace simple_grasp
{

// environment
static const std::string SUPPORT_SURFACE1_NAME = "door_handle";


// handle dimensions
static const double HANDLE_LENGTH = 0.15;
static const double HANDLE_RADIUS = 0.01;


inline void createEnvironment(moveit_visual_tools::VisualToolsPtr visual_tools_)
{
    visual_tools_->cleanupCO(SUPPORT_SURFACE1_NAME);


    // --------------------------------------------------------------------------------------------
    // Add objects to scene

    // Tables                          x,         y,          angle, width,       height,       depth,       name
    geometry_msgs::Pose handle_pose;
    handle_pose.orientation.w = 0.640901340307;
    handle_pose.orientation.x = -0.662572304528;
    handle_pose.orientation.y = 0.235059353049;
    handle_pose.orientation.z =  -0.30820531113;
    handle_pose.position.x = 0.958264405468;
    handle_pose.position.y = -0.378703808557;
    handle_pose.position.z = 0.150069256758;
//    visual_tools_->publishCollisionCylinder(handle_pose,SUPPORT_SURFACE1_NAME, 0.01, 0.1);



}



} // namespace

#endif
