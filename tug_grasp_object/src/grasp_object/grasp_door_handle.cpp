
#include <tug_grasp_object/grasp_object/grasp_door_handle.h>
#include <tug_grasp_object/grasping_point/grasping_point_door_handle_calculation.h>

namespace grasp_object
{
GraspDoorHandle::GraspDoorHandle(std::string name, geometry_msgs::PoseStamped pose, std::string arm) :
    GraspObject(name, pose, arm),
    gripper_closed_(false)
{
    grasping_point_calculation_ptr_ = boost::make_shared<grasping_point::GraspingPointDoorHandle>();
    ros::NodeHandle nh;

    gripper_state_sub_ = nh.subscribe<baxter_core_msgs::EndEffectorState>("/robot/end_effector/" + arm
                         + "_gripper/state",
                         1, &GraspDoorHandle::stateCallback, this);
}

GraspDoorHandle::~GraspDoorHandle()
{}

bool GraspDoorHandle::graspObject()
{
    grasping_point_calculation_ptr_->calculateGraspingPoints(object_pose_, data_, possible_grasps_);
    ROS_ERROR_STREAM("NUmber of generated grasps " << possible_grasps_.size());

    move_group_->pick(object_name_, possible_grasps_);
    ROS_ERROR("pick finished");
    sleep(5.0);
    ROS_ERROR_STREAM("result of pick "<< gripper_closed_);


    return gripper_closed_;

}

void GraspDoorHandle::stateCallback(const baxter_core_msgs::EndEffectorStateConstPtr& msg)
{


  // Check for errors every 50 refreshes
  if(msg->gripping)
  {
    gripper_closed_ = true;
  }
}

}
