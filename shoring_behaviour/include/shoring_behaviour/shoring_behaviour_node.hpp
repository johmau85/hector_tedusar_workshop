/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Johannes Maurer
 *                      Institute for Software Technology,
 *                      Graz University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef shoring_bahaviour_node_hpp___
#define shoring_bahaviour_node_hpp__

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <shoring_msgs/PlaceBlockAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

namespace shoring_bahaviour
{

typedef actionlib::SimpleActionServer<shoring_msgs::PlaceBlockAction> PlaceBlockActionActionServer;
typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;

class ShoringBehaviour
{
private:
    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;

    boost::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;

    PlaceBlockActionActionServer place_block_action_server_;

    GripperCommandActionClient gripper_command_action_client_;

    ros::Publisher display_publisher_;

    std::vector<geometry_msgs::PoseStamped> place_poses_;
    int block_count_;

    std::string arm_base_link_;

    double tool_distance_;

public:
    ShoringBehaviour();

    virtual ~ShoringBehaviour();

    void init();

    void executePlaceBlockCB(const shoring_msgs::PlaceBlockGoalConstPtr &goal);

private:

    void moveToPose(geometry_msgs::Pose pose);

    void moveArmCartesian(geometry_msgs::Pose point);

    void openGripper();

};

} // end namespace shoring_bahaviour

#endif // shoring_bahaviour_node_hpp___
