#ifndef GRASPING_POINT_OBJECT_CALCULATION_
#define GRASPING_POINT_OBJECT_CALCULATION_

#include <moveit_msgs/Grasp.h>
#include <ros/ros.h>
#include <tug_grasp_object/grasping_point/data/data.h>
#include <geometry_msgs/PoseStamped.h>

// TF
#include <tf_conversions/tf_eigen.h>

// Msgs
#include <geometry_msgs/PoseArray.h>

// MoveIt
#include <moveit_msgs/Grasp.h>
#include <moveit/macros/deprecation.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Visualization
#include <moveit_visual_tools/visual_tools.h>

// C++
#include <math.h>

namespace grasping_point
{

class GraspingPointObjectCalculation
{

public:
    inline GraspingPointObjectCalculation(){}
    inline virtual ~GraspingPointObjectCalculation(){}

    virtual void calculateGraspingPoints(geometry_msgs::PoseStamped object_pose, GraspData data, std::vector<moveit_msgs::Grasp>& grasps) = 0;
};


}

#endif
