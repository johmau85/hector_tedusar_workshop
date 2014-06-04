/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Johannes Maurer, Alexander Buchegger
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

#ifndef weiss_wsg_gripper_control_hpp___
#define weiss_wsg_gripper_control_hpp___

#include <boost/shared_ptr.hpp>
#include <hardware_interface/robot_hw.h>
#include <weiss_wsg_gripper/gripper.h>

namespace schunk_lwa4p_control
{

class WeissWsgGripperControl
{
public:
    struct Parameters {
        std::string joint_name_left_;
        std::string joint_name_right_;
        int can_id_;
        double default_velocity_;
        double default_effort_;
    };

    WeissWsgGripperControl();

    void initialize(hardware_interface::RobotHW & robot_hw, const Parameters & parameters);
    void read();
    void write();

private:
    struct JointInfo
    {
        JointInfo();

        double current_position_;
        double current_velocity_;
        double current_effort_;
        double commanded_position_;
    };

    void registerJoint(hardware_interface::RobotHW & robot_hw, const std::string & name, JointInfo & joint_info);

    Parameters parameters_;
    JointInfo left_joint_;
    JointInfo right_joint_;
    boost::shared_ptr<weiss_wsg_gripper::Gripper> gripper_;
};

}

#endif // weiss_wsg_gripper_control_hpp___
