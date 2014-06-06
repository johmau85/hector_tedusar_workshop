#ifndef SIMPLE_GRASP_
#define SIMPLE_GRASP_
// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h> // simple tool for showing graspsp

// Baxter specific properties
#include <moveit_simple_grasps/grasp_data.h>
#include <grasp_handle/environment/door_push.h>

namespace simple_grasp
{



struct MetaObject
{
  std::string name;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
};

class Grasp
{
private:
  //good pregrasp pose
  /*
pose:
  position:
    x: 0.870204301546
    y: -0.368756868089
    z: 0.134338293486
  orientation:
    x: 0.493102788396
    y: -0.528141008527
    z: 0.547029919017
    w: -0.422699636727

    */
  // grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

  // settings
  bool auto_reset_;
  int auto_reset_sec_;

public:

  Grasp();
  ~Grasp();


  void resetBlock(MetaObject block);
  MetaObject createStartBlock(double x, double y, const std::string name);

  bool pick(std::string name);
  bool place(const geometry_msgs::Pose& goal_block_pose, std::string block_name);
  bool startRoutine(std::string name);
  bool promptUser(std::string string);

};
}
#endif
