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

#include <shoring_behaviour/shoring_behaviour_node.hpp>

namespace shoring_bahaviour
{

ShoringBehaviour::ShoringBehaviour() :
    nh_(),
    place_block_action_server_(nh_, "place_block", boost::bind(&ShoringBehaviour::executePlaceBlockCB, this, _1), false),
    gripper_command_action_client_("/gripper_controller/gripper_cmd", true),
    block_count_(0)
{

    place_block_action_server_.start();
}

ShoringBehaviour::~ShoringBehaviour()
{

}

void ShoringBehaviour::init()
{
    ros::NodeHandle private_nh("~");

    std::string group;
    private_nh.param<std::string>("group", group, std::string("lwa4p_arm"));

    private_nh.param<double>("tool_distance", tool_distance_, 0.2);

    move_group_.reset(new moveit::planning_interface::MoveGroup(group));

    ROS_INFO_STREAM("Planning reference frame: " << move_group_->getPlanningFrame());
    arm_base_link_ = move_group_->getPlanningFrame();
    ROS_INFO_STREAM("End effector frame: " << move_group_->getEndEffectorLink());

    // TODO: read place poses from config
    geometry_msgs::PoseStamped place_pose;
    place_pose.header.frame_id = std::string("tower");
    place_pose.header.stamp = ros::Time(0);

    // first layer
    place_pose.pose.position.x = -0.11;
    place_pose.pose.position.y =  0.0;
    place_pose.pose.position.z =  tool_distance_ + 0.06;
    place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI, -M_PI/2);
    place_poses_.push_back(place_pose);

    place_pose.pose.position.x =  0.11;
    place_pose.pose.position.y =  0.0;
    place_poses_.push_back(place_pose);

    // second layer
    place_pose.pose.position.x =  0.0;
    place_pose.pose.position.y = -0.11;
    place_pose.pose.position.z += 0.10;
    place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI, 0.0);
    place_poses_.push_back(place_pose);

    place_pose.pose.position.x =  0.0;
    place_pose.pose.position.y =  0.11;
    place_poses_.push_back(place_pose);

    // third layer
    place_pose.pose.position.x = -0.11;
    place_pose.pose.position.y =  0.0;
    place_pose.pose.position.z += 0.10;
    place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI, -M_PI/2);
    place_poses_.push_back(place_pose);

    place_pose.pose.position.x =  0.11;
    place_pose.pose.position.y =  0.0;
    place_poses_.push_back(place_pose);

    // fourth layer
    place_pose.pose.position.x =  0.0;
    place_pose.pose.position.y = -0.11;
    place_pose.pose.position.z += 0.10;
    place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI, 0.0);
    place_poses_.push_back(place_pose);

    place_pose.pose.position.x =  0.0;
    place_pose.pose.position.y =  0.11;
    place_poses_.push_back(place_pose);

    display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

}

void ShoringBehaviour::executePlaceBlockCB(const shoring_msgs::PlaceBlockGoalConstPtr &goal)
{
    ROS_INFO("Start Place Block");

    geometry_msgs::PoseStamped pose_stamped;
    tf_listener_.transformPose(arm_base_link_, place_poses_[block_count_++], pose_stamped);
    geometry_msgs::Pose place_pose = pose_stamped.pose;

    geometry_msgs::Pose pre_place_pose;
    pre_place_pose = place_pose;
    pre_place_pose.position.z += 0.2;

    ROS_INFO_STREAM("pre_place_pose:" << std::endl << pre_place_pose);
    ROS_INFO_STREAM("place_pose:" << std::endl << place_pose);

    moveToPose(pre_place_pose);

    if (place_block_action_server_.isPreemptRequested() || !ros::ok())
    {
        place_block_action_server_.setPreempted();
        return;
    }

    // move arm down
    moveArmCartesian(place_pose);

    if (place_block_action_server_.isPreemptRequested() || !ros::ok())
    {
        place_block_action_server_.setPreempted();
        return;
    }

    openGripper();

    if (place_block_action_server_.isPreemptRequested() || !ros::ok())
    {
        place_block_action_server_.setPreempted();
        return;
    }

    // move arm up
    moveArmCartesian(pre_place_pose);

    shoring_msgs::PlaceBlockResult result;
    place_block_action_server_.setSucceeded(result);

    ROS_INFO("Start Place Block Succeeded");
}


void ShoringBehaviour::moveToPose(geometry_msgs::Pose pose)
{
    move_group_->setPoseTarget(pose);


    moveit::planning_interface::MoveGroup::Plan my_plan;
    while(!move_group_->plan(my_plan))
    {
        if (place_block_action_server_.isPreemptRequested() || !ros::ok())
        {
            place_block_action_server_.setPreempted();
            return;
        }
    }

    /* Uncomment below line when working with a real robot*/
    move_group_->execute(my_plan);
    move_group_->clearPoseTarget();

    sleep(0.1);
}

void ShoringBehaviour::openGripper()
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 10.0;
    goal.command.max_effort = 0.0;
    gripper_command_action_client_.sendGoal(goal);

    while(!gripper_command_action_client_.waitForResult(ros::Duration(5.0)))
    {
        if (place_block_action_server_.isPreemptRequested() | !ros::ok())
        {
            place_block_action_server_.setPreempted();
            return;
        }
    }
}

void ShoringBehaviour::moveArmCartesian(geometry_msgs::Pose point)
{
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(point);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints,
                                                        0.02,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.01);

    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    moveit_msgs::RobotTrajectory smooth_trajectory;
    rt.getRobotTrajectoryMsg(smooth_trajectory);

    // Finally plan and execute the trajectory
    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = smooth_trajectory;

    move_group_->execute(plan);
    sleep(0.1);
}

} // end namespace shoring_bahaviour



int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "arm_scan_area");

        // start a ROS spinning thread
        ros::AsyncSpinner spinner(2);
        spinner.start();

        shoring_bahaviour::ShoringBehaviour scan_node;

        scan_node.init();

        ros::spin();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
