#include <schunk_lwa4p_control/weiss_wsg_gripper_control.hpp>
#include <stdexcept>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <weiss_wsg_gripper/gripper_exceptions.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace schunk_lwa4p_control
{

WeissWsgGripperControl::Parameters::Parameters()
    : can_id_(-1), default_velocity_(0.1), default_effort_(10.0), command_timeout_(0.1)
{
}

WeissWsgGripperControl::JointInfo::JointInfo()
    : current_position_(0), current_velocity_(0), current_effort_(0), commanded_position_(0)
{
}

WeissWsgGripperControl::WeissWsgGripperControl()
    : last_commanded_position_(NAN), state_(STATE_IDLE), command_transmitted_time_(), current_position_(0), current_velocity_(0), current_effort_(0)
{
}

void WeissWsgGripperControl::initialize(hardware_interface::RobotHW & robot_hw, const Parameters & parameters)
{
    parameters_ = parameters;

    registerJoint(robot_hw, parameters.joint_name_left_, left_joint_);
    registerJoint(robot_hw, parameters.joint_name_right_, right_joint_);

    std::ostringstream ss;
    ss << "ipa_canopen:" << parameters.can_id_;
    ROS_INFO("creating gripper instance with transport URI %s", ss.str().c_str());
    gripper_ = boost::make_shared<weiss_wsg_gripper::Gripper>(ss.str(), false, 0.3f);
    ROS_INFO("connecting to gripper...");
    gripper_->connect();
    ROS_INFO("acknowledging fast stop...");
    gripper_->acknowledgeFastStop();
    ROS_INFO("setting force limit...");
    gripper_->setForceLimit(parameters.default_effort_);
    ROS_INFO("executing homing sequence...");
    gripper_->executeHomingSequence(weiss_wsg_gripper::Gripper::HOMING_DIRECTION_DEFAULT);

    ROS_INFO("registering regular state updates...");
    gripper_->getOpeningWidth(weiss_wsg_gripper::Gripper::AUTO_UPDATE_ALWAYS, 100);
    gripper_->getSpeed(weiss_wsg_gripper::Gripper::AUTO_UPDATE_ALWAYS, 100);
    gripper_->getForce(weiss_wsg_gripper::Gripper::AUTO_UPDATE_ALWAYS, 100);

    gripper_->registerAsyncPacketCallback(boost::bind(&WeissWsgGripperControl::handleAsyncPacket, this, _1));
}

void WeissWsgGripperControl::cleanup()
{
    if (gripper_)
    {
        gripper_->stop();
        gripper_->getOpeningWidth(weiss_wsg_gripper::Gripper::AUTO_UPDATE_DISABLED, 0);
        gripper_->getSpeed(weiss_wsg_gripper::Gripper::AUTO_UPDATE_DISABLED, 0);
        gripper_->getForce(weiss_wsg_gripper::Gripper::AUTO_UPDATE_DISABLED, 0);
    }
}

void WeissWsgGripperControl::read()
{
    //ROS_INFO("%s: before gripper_->checkForAsyncPackets", __func__);
    gripper_->checkForAsyncPackets();
    //ROS_INFO("%s: after gripper_->checkForAsyncPackets", __func__);

    boost::lock_guard<boost::mutex> lock(current_state_lock_);

    left_joint_.current_position_ = -current_position_ / 2.0;
    right_joint_.current_position_ = current_position_ / 2.0;

    left_joint_.current_velocity_ = -current_velocity_ / 2.0;
    right_joint_.current_velocity_ = current_velocity_ / 2.0;

    left_joint_.current_effort_ = -current_effort_;
    right_joint_.current_effort_ = current_effort_;
}

void WeissWsgGripperControl::write()
{
    double commanded_position = left_joint_.commanded_position_ + right_joint_.commanded_position_;

    boost::lock_guard<boost::mutex> lock(current_state_lock_);
    switch (state_)
    {
    case STATE_IDLE:
        if (commanded_position != last_commanded_position_)
        {
            if (transmitMoveCommand(commanded_position))
            {
                last_commanded_position_ = commanded_position;
                state_ = STATE_WAITING_FOR_MOVE_COMMAND_RESULT;
            }
        }
        break;

    case STATE_WAITING_FOR_MOVE_COMMAND_RESULT:
        // Retransmit command on timeout:
        if ((ros::Time::now() - command_transmitted_time_) > parameters_.command_timeout_)
        {
            if (transmitMoveCommand(commanded_position))
                last_commanded_position_ = commanded_position;
        }
        break;

    case STATE_MOVING:
        if (commanded_position != last_commanded_position_)
        {
            if (transmitStopCommand())
                state_ = STATE_WAITING_FOR_STOP_COMMAND_RESULT;
        }
        break;

    case STATE_WAITING_FOR_STOP_COMMAND_RESULT:
        // Retransmit command on timeout:
        if ((ros::Time::now() - command_transmitted_time_) > parameters_.command_timeout_)
            transmitStopCommand();
        break;

    case STATE_FAST_STOP:
        ROS_WARN("Gripper in FAST STOP mode; trying to release it");
        gripper_->acknowledgeFastStop();
        state_ = STATE_IDLE;
        break;
    }
}

bool WeissWsgGripperControl::transmitMoveCommand(double commanded_position)
{
    try
    {
        gripper_->transmitPrepositionFingersCommand(weiss_wsg_gripper::Gripper::BLOCKING_CLAMP_ON_BLOCK,
                                     weiss_wsg_gripper::Gripper::MOVEMENT_TYPE_ABSOLUTE,
                                     static_cast<float>(commanded_position * 1000.0),
                                     static_cast<float>(parameters_.default_velocity_ * 1000.0 * 2.0));
        command_transmitted_time_ = ros::Time::now();
        return true;
    }
    catch (weiss_wsg_gripper::GripperException & ex)
    {
        ROS_ERROR("Gripper control: error while writing position: %s", ex.what());
    }
    return false;
}

bool WeissWsgGripperControl::transmitStopCommand()
{
    try
    {
        gripper_->transmitStopCommand();
        command_transmitted_time_ = ros::Time::now();
        return true;
    }
    catch (weiss_wsg_gripper::GripperException & ex)
    {
        ROS_ERROR("Gripper control: error while writing position: %s", ex.what());
    }
    return false;
}

void WeissWsgGripperControl::registerJoint(hardware_interface::RobotHW & robot_hw, const std::string & name, JointInfo & joint_info)
{
    hardware_interface::JointStateInterface * joint_state_interface = robot_hw.get<hardware_interface::JointStateInterface>();
    hardware_interface::PositionJointInterface * joint_position_interface = robot_hw.get<hardware_interface::PositionJointInterface>();
    if (joint_state_interface == NULL || joint_position_interface == NULL)
        throw std::runtime_error("robot_hw must contain a JointStateInterface and a PositionJointInterface, but doesn't");

    joint_state_interface->registerHandle(hardware_interface::JointStateHandle(name, &joint_info.current_position_,
                                                                               &joint_info.current_velocity_, &joint_info.current_effort_));
    joint_position_interface->registerHandle(hardware_interface::JointHandle(joint_state_interface->getHandle(name),
                                                                             &joint_info.commanded_position_));
}

void WeissWsgGripperControl::handleAsyncPacket(const weiss_wsg_gripper::Gripper::AsyncPacketInfo & packet)
{
    boost::lock_guard<boost::mutex> lock(current_state_lock_);

    //ROS_INFO("GripperControl: received async packet: %s", weiss_wsg_gripper::Gripper::getCommandName(packet.command_).c_str());

    if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_ACCESS_DENIED)
    {
        state_ = STATE_FAST_STOP;
    }
    else
    {
        switch (packet.command_)
        {
        case weiss_wsg_gripper::Gripper::CMD_PREPOSITION_FINGERS:
            switch (state_) {
            case STATE_IDLE:
            case STATE_WAITING_FOR_STOP_COMMAND_RESULT:
            case STATE_FAST_STOP:
                // Can't happen -- should be safe to ignore this.
                break;

            case STATE_WAITING_FOR_MOVE_COMMAND_RESULT:
                if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_CMD_PENDING)
                    state_ = STATE_MOVING;
                else if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_SUCCESS ||
                         packet.error_code_ == weiss_wsg_gripper::Gripper::E_AXIS_BLOCKED ||
                         packet.error_code_ == weiss_wsg_gripper::Gripper::E_RANGE_ERROR)
                    // Can't happen -- let's assume the E_CMD_PENDING packet got lost and we're already there.
                    state_ = STATE_IDLE;
                else
                {
                    // Other error -- wait for timeout to send command again.
                }
                break;

            case STATE_MOVING:
                if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_SUCCESS ||
                    packet.error_code_ == weiss_wsg_gripper::Gripper::E_AXIS_BLOCKED ||
                    packet.error_code_ == weiss_wsg_gripper::Gripper::E_RANGE_ERROR)
                    state_ = STATE_IDLE;
                else
                {
                    // Other error -- ignore until next change of commanded_position in write method.
                }
                break;
            }
            break;

        case weiss_wsg_gripper::Gripper::CMD_STOP:
            switch (state_)
            {
            case STATE_IDLE:
            case STATE_WAITING_FOR_MOVE_COMMAND_RESULT:
            case STATE_MOVING:
            case STATE_FAST_STOP:
                // Can't happen -- should be safe to ignore this.
                break;

            case STATE_WAITING_FOR_STOP_COMMAND_RESULT:
                // Ok; wait for write method to send next command. Errors are ignored to return to a safe state.
                state_ = STATE_IDLE;
                break;
            }
            break;

        case weiss_wsg_gripper::Gripper::CMD_GET_OPENING_WIDTH:
            if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_SUCCESS)
                current_position_ = packet.payload_.as_float_ / 1000.0;
            break;

        case weiss_wsg_gripper::Gripper::CMD_GET_SPEED:
            if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_SUCCESS)
                current_velocity_ = packet.payload_.as_float_ / 1000.0;
            break;

        case weiss_wsg_gripper::Gripper::CMD_GET_FORCE:
            if (packet.error_code_ == weiss_wsg_gripper::Gripper::E_SUCCESS)
                current_effort_ = packet.payload_.as_float_;
            break;
        }
    }
}


}





