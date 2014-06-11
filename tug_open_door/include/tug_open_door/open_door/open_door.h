#ifndef OPEN_DOOR_
#define OPEN_DOOR_


#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/macros/deprecation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

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

namespace open_door
{
class OpenDoor
{
public:
    OpenDoor();
    virtual ~OpenDoor();
    ros::Publisher display_publisher_;
    ros::Publisher display_way_points_;
    ros::Publisher display_mid_point_;

    bool openDoor(geometry_msgs::PoseStamped midpoint, float radius, std::string direction);
private:
    boost::shared_ptr<move_group_interface::MoveGroup> move_group_;
};


}


#endif
