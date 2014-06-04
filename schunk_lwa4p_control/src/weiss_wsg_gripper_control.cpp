#include <schunk_lwa4p_control/weiss_wsg_gripper_control.hpp>
#include <stdexcept>
#include <sstream>
#include <boost/make_shared.hpp>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace schunk_lwa4p_control
{

WeissWsgGripperControl::WeissWsgGripperControl()
    : current_position_(0), current_velocity_(0), current_effort_(0), commanded_position_(0)
{
}

void WeissWsgGripperControl::initialize(hardware_interface::RobotHW & robot_hw, const Parameters & parameters)
{
    parameters_ = parameters;

    hardware_interface::JointStateInterface * joint_state_interface = robot_hw.get<hardware_interface::JointStateInterface>();
    hardware_interface::PositionJointInterface * joint_position_interface = robot_hw.get<hardware_interface::PositionJointInterface>();
    if (joint_state_interface == NULL || joint_position_interface == NULL)
        throw std::runtime_error("robot_hw must contain a JointStateInterface and a PositionJointInterface, but doesn't");

    joint_state_interface->registerHandle(hardware_interface::JointStateHandle(parameters.joint_name_, &current_position_,
                                                                               &current_velocity_, &current_effort_));
    joint_position_interface->registerHandle(hardware_interface::JointHandle(joint_state_interface->getHandle(parameters.joint_name_),
                                                                             &commanded_position_));

    std::ostringstream ss;
    ss << "ipa_canopen:" << parameters.can_id_;
    gripper_ = boost::make_shared<weiss_wsg_gripper::Gripper>(ss.str());
    gripper_->acknowledgeFastStop();
    gripper_->setForceLimit(parameters.default_effort_);
    gripper_->executeHomingSequence(weiss_wsg_gripper::Gripper::HOMING_DIRECTION_DEFAULT);
}

void WeissWsgGripperControl::read()
{
    current_position_ = gripper_->getOpeningWidth(weiss_wsg_gripper::Gripper::AUTO_UPDATE_DISABLED, 0) / 1000.0 / 2.0;
    current_velocity_ = gripper_->getSpeed(weiss_wsg_gripper::Gripper::AUTO_UPDATE_DISABLED, 0) / 1000.0 / 2.0;
    current_effort_ = gripper_->getForce(weiss_wsg_gripper::Gripper::AUTO_UPDATE_DISABLED, 0);
}

void WeissWsgGripperControl::write()
{
    gripper_->prepositionFingers(weiss_wsg_gripper::Gripper::BLOCKING_CLAMP_ON_BLOCK, weiss_wsg_gripper::Gripper::MOVEMENT_TYPE_ABSOLUTE,
                                 static_cast<float>(commanded_position_ * 1000.0 * 2.0),
                                 static_cast<float>(parameters_.default_velocity_ * 1000.0 * 2.0));
}


}

