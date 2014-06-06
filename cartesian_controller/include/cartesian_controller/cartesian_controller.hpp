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

#ifndef cartesian_controller_hpp___
#define cartesian_controller_hpp___

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace velocity_controllers
{

class CartesianController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
    bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &nh);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

private:
    hardware_interface::JointHandle joint_;
    ros::Subscriber vel_cmd_sub_;

    KDL::Chain kdl_chain_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    // KDL Solvers performing the actual computations
    boost::scoped_ptr<KDL::ChainFkSolverPos>    joint_to_pose_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> joint_to_jacobian_solver_;

    // The variables (which need to be pre-allocated).
    KDL::JntArray  q0_;           // Joint initial positions
    KDL::JntArrayVel  jnt_posvel_;      // Joint positions and velocities

    KDL::JntArray joint_vel_;

    KDL::Frame     x_;            // Tip pose
    KDL::Frame     xd_;           // Tip desired pose
    KDL::Frame     x0_;           // Tip initial pose

    KDL::Twist     xerr_;         // Cart error
    KDL::Twist  cmd_value_;
    KDL::Twist     xdot_;         // Cart velocity
    KDL::Wrench    F_;            // Cart effort
    KDL::Jacobian  J_;            // Jacobian

    void velCmdCB(const geometry_msgs::TwistConstPtr& msg);
};

}

#endif // cartesian_controller_hpp___
