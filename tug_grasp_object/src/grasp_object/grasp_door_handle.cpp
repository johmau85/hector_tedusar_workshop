
#include <tug_grasp_object/grasp_object/grasp_door_handle.h>
#include <tug_grasp_object/grasping_point/grasping_point_door_handle_calculation.h>

namespace grasp_object
{
GraspDoorHandle::GraspDoorHandle(std::string name, geometry_msgs::PoseStamped pose, std::string arm) :
    GraspObject(name, pose, arm)
{
    grasping_point_calculation_ptr_ = boost::make_shared<grasping_point::GraspingPointDoorHandle>();
}

GraspDoorHandle::~GraspDoorHandle()
{}

bool GraspDoorHandle::graspObject()
{
    grasping_point_calculation_ptr_->calculateGraspingPoints(object_pose_, data_, possible_grasps_);
    ROS_ERROR_STREAM("NUmber of generated grasps " << possible_grasps_.size());

    bool result =  move_group_->pick(object_name_, possible_grasps_);
    ROS_ERROR_STREAM("result of pick "<< result);
    return result;

}

}
