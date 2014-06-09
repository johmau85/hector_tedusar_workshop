#ifndef GRASP_OBJECT_
#define GRASP_OBJECT_

#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/Grasp.h>
#include <moveit/macros/deprecation.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_visual_tools/visual_tools.h>
#include <moveit/move_group_interface/move_group.h>

#include <boost/shared_ptr.hpp>

#include <math.h>
#define _USE_MATH_DEFINES

#include <tug_grasp_object/grasping_point/data/data.h>
#include <tug_grasp_object/grasping_point/grasping_point_object_calculation.h>

#include <boost/make_shared.hpp>

namespace grasp_object
{

class GraspObject
{
public:
    GraspObject(std::string name, geometry_msgs::PoseStamped pose, std::string arm);
    virtual ~GraspObject();

    virtual bool graspObject() = 0;
protected:

    std::string object_name_;
    geometry_msgs::PoseStamped object_pose_;
    std::string arm_;
    boost::shared_ptr<move_group_interface::MoveGroup> move_group_;
    moveit_visual_tools::VisualToolsPtr visual_tools_;


    grasping_point::GraspData data_;
    std::vector<moveit_msgs::Grasp> possible_grasps_;

    boost::shared_ptr<grasping_point::GraspingPointObjectCalculation> grasping_point_calculation_ptr_;



};

typedef boost::shared_ptr<GraspObject> GraspObjectPtr;

}



#endif
