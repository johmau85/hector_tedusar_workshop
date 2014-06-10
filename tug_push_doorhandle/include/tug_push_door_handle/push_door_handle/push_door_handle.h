#ifndef PUSH_DOOR_HANDLE_
#define PUSH_DOOR_HANDLE_

#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/macros/deprecation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <Eigen/Core>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_visual_tools/visual_tools.h>
#include <moveit/move_group_interface/move_group.h>
#include <baxter_core_msgs/EndpointState.h>

#include <boost/shared_ptr.hpp>

#include <math.h>
#define _USE_MATH_DEFINES


#include <boost/make_shared.hpp>

namespace push_door_handle
{
class PushDoorHandle
{
public:
    PushDoorHandle();
    virtual ~PushDoorHandle();

    ros::Publisher display_publisher_;
//    ros::Publisher desired_pose_pub_;
//    ros::Subscriber current_right_hand_pose_;
    bool pushHandle();
//    void stateCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
private:
    boost::shared_ptr<move_group_interface::MoveGroup> move_group_;
//    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped desired_pose_;


};

}

#endif
