#include<tug_grasp_object/grasp_object/grasp_object.h>

namespace grasp_object
{

GraspObject::GraspObject(std::string name, geometry_msgs::PoseStamped pose, std::string arm):
    object_name_(name),
    object_pose_(pose),
    arm_(arm)
{
    ros::NodeHandle nh;

    if (!data_.loadRobotGraspData(nh, arm_+"_hand"))
            ros::shutdown();

    std::string planning_group_name = arm_+"_arm";
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
    move_group_->setPlanningTime(180.0);



    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools( data_.base_link_));
    visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->loadEEMarker(data_.ee_group_, planning_group_name);

    visual_tools_->setMuted(true);

}

GraspObject::~GraspObject()
{}

}
