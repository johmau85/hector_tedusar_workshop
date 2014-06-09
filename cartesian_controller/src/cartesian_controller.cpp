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

#include <cartesian_controller/cartesian_controller.hpp>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>

PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianController, controller_interface::ControllerBase)

namespace velocity_controllers
{

bool CartesianController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &nh)
{
    std::string robot_description, root_name, tip_name;

    if(!nh.searchParam("robot_description", robot_description))
    {
        ROS_ERROR("Failed to get robot_description parameter");
        return false;
    }

    if (!nh.getParam("root_name", root_name))
    {
        ROS_ERROR("Failed to get root_name parameter");
        return false;
    }

    if (!nh.getParam("tip_name", tip_name))
    {
        ROS_ERROR("Failed to get tip_name parameter");
        return false;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromParam(robot_description, kdl_tree)){
        ROS_ERROR("Failed to construct kdl tree from robot description parameter");
        return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree.");
        return false;
    }


    // Get joint handles for all of the joints in the chain
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
        joint_handles_.push_back(hw->getHandle(it->getJoint().getName()));
    }

    // Construct the kdl solvers in non-realtime.
    joint_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    joint_to_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resize (pre-allocate) the variables in non-realtime.
    jnt_posvel_.resize(kdl_chain_.getNrOfJoints());
    joint_vel_.resize(kdl_chain_.getNrOfJoints());
    q0_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    vel_cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("vel_cmd", 1, &CartesianController::velCmdCB, this);

    return true;
}

void CartesianController::update(const ros::Time& time, const ros::Duration& period)
{
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
        jnt_posvel_.q(i) = joint_handles_[i].getPosition();
        jnt_posvel_.qdot(i) = joint_handles_[i].getVelocity();
    }

    // Compute the forward kinematics and Jacobian (at this location).
    joint_to_pose_solver_->JntToCart(jnt_posvel_.q, x_);
    joint_to_jacobian_solver_->JntToJac(jnt_posvel_.q, J_);

    // Convert the wrench into joint efforts
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
    {
        joint_vel_(i) = 0;
        for (unsigned int j=0; j<6; j++) {
            joint_vel_(i) += (J_(j,i) * cmd_value_(j));
        }

        // Set the joint effort
        joint_handles_[i].setCommand(joint_vel_(i));
    }
}

void CartesianController::starting(const ros::Time& time)
{
    // Get the current joint values to compute the initial tip location.
    // get joint positions
    for(size_t i=0; i<joint_handles_.size(); i++)
    {
        q0_(i) = joint_handles_[i].getPosition();
    }
    joint_to_pose_solver_->JntToCart(q0_, x0_);

    cmd_value_[0] = 0.0;
    cmd_value_[1] = 0.0;
    cmd_value_[2] = 0.0;
    cmd_value_[3] = 0.0;
    cmd_value_[4] = 0.0;
    cmd_value_[5] = 0.0;

}

void CartesianController::stopping(const ros::Time& time)
{

}

void CartesianController::velCmdCB(const geometry_msgs::TwistConstPtr& msg)
{
    cmd_value_[0] = msg->linear.x;
    cmd_value_[1] = msg->linear.y;
    cmd_value_[2] = msg->linear.z;
    cmd_value_[3] = msg->angular.x;
    cmd_value_[4] = msg->angular.y;
    cmd_value_[5] = msg->angular.z;
}

}
