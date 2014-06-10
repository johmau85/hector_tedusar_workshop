#include <schunk_teleop/schunk_teleop_node.h>
#include <stdexcept>

namespace schunk_teleop
{

SchunkTeleopNode::SchunkTeleopNode()
    : repeat_vel_commands_(false)
{
}

void SchunkTeleopNode::init()
{
    loadParameters();
    ROS_INFO("Controller: %s",parameters_.controller_name_.c_str());

    cmd_vel_.header.frame_id = parameters_.frame_id_;

    cmd_generator_timer_ = nh_.createTimer(ros::Duration(1.0 / parameters_.publication_rate_),
        &SchunkTeleopNode::cmdGeneratorTimerCB, this, false);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cartesian_controller/arm_cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
        &SchunkTeleopNode::joyCB, this);

    if (parameters_.have_gripper_)
        gripper_command_ac_.reset(new GripperCommandActionClient("/gripper_controller/gripper_cmd", true));
}

void SchunkTeleopNode::loadParameters()
{
    ros::NodeHandle private_nh("~");

    getOptionalParameter(private_nh, "publication_rate", parameters_.publication_rate_, 20.0);
    getRequiredParameter(private_nh, "frame_id", parameters_.frame_id_);

    getOptionalParameter(private_nh, "std_translation_vel", parameters_.std_translation_vel_, 0.01);
    getOptionalParameter(private_nh, "std_rotation_vel", parameters_.std_rotation_vel_, 0.1);

    getOptionalParameter(private_nh, "have_gripper", parameters_.have_gripper_, true);
    getOptionalParameter(private_nh, "gripper_grasp_position", parameters_.gripper_grasp_position_, 0.0);
    getOptionalParameter(private_nh, "gripper_grasp_effort", parameters_.gripper_grasp_effort_, 10.0);
    getOptionalParameter(private_nh, "gripper_release_position", parameters_.gripper_release_position_, 0.1);
    getOptionalParameter(private_nh, "gripper_release_effort", parameters_.gripper_release_effort_, 0.0);

    getRequiredParameter(private_nh, "controller", parameters_.controller_name_);
    getRequiredParameter(private_nh, "button_translation_deadman_switch", parameters_.button_translation_deadman_switch_id_);
    getRequiredParameter(private_nh, "button_rotation_deadman_switch", parameters_.button_rotation_deadman_switch_id_);
    getRequiredParameter(private_nh, "button_turbo", parameters_.button_turbo_id_);

    if (parameters_.have_gripper_)
    {
        getRequiredParameter(private_nh, "button_gripper_grasp", parameters_.button_gripper_grasp_id_);
        getRequiredParameter(private_nh, "button_gripper_release", parameters_.button_gripper_release_id_);
    }

    getRequiredParameter(private_nh, "axis_translation_x", parameters_.axis_translation_x_id_);
    getRequiredParameter(private_nh, "axis_translation_y", parameters_.axis_translation_y_id_);
    getRequiredParameter(private_nh, "axis_translation_z", parameters_.axis_translation_z_id_);
    getRequiredParameter(private_nh, "axis_rotation_x", parameters_.axis_rotation_x_id_);
    getRequiredParameter(private_nh, "axis_rotation_y", parameters_.axis_rotation_y_id_);
    getRequiredParameter(private_nh, "axis_rotation_z", parameters_.axis_rotation_z_id_);
}

void SchunkTeleopNode::cmdGeneratorTimerCB(const ros::TimerEvent & e)
{
    if (repeat_vel_commands_)
        sendCmdVel();
}

void SchunkTeleopNode::joyCB(const sensor_msgs::Joy::ConstPtr & joy)
{
    bool have_vel_commands = false;

    double turbo = joy->buttons.at(parameters_.button_turbo_id_) ? 2 : 1;

    if (joy->buttons.at(parameters_.button_translation_deadman_switch_id_) && !joy->buttons.at(parameters_.button_rotation_deadman_switch_id_))
    {
      have_vel_commands = true;
      cmd_vel_.twist.linear.x = parameters_.std_translation_vel_ * turbo * joy->axes.at(parameters_.axis_translation_x_id_);
      cmd_vel_.twist.linear.y = parameters_.std_translation_vel_ * turbo * joy->axes.at(parameters_.axis_translation_y_id_);
      cmd_vel_.twist.linear.z = parameters_.std_translation_vel_ * turbo * joy->axes.at(parameters_.axis_translation_z_id_);
    }
    else
    {
      cmd_vel_.twist.linear.x = 0;
      cmd_vel_.twist.linear.y = 0;
      cmd_vel_.twist.linear.z = 0;
    }

    if (!joy->buttons.at(parameters_.button_translation_deadman_switch_id_) && joy->buttons.at(parameters_.button_rotation_deadman_switch_id_))
    {
        have_vel_commands = true;
        cmd_vel_.twist.angular.x = parameters_.std_rotation_vel_ * turbo * joy->axes.at(parameters_.axis_rotation_x_id_);
        cmd_vel_.twist.angular.y = parameters_.std_rotation_vel_ * turbo * joy->axes.at(parameters_.axis_rotation_y_id_);
        cmd_vel_.twist.angular.z = parameters_.std_rotation_vel_ * turbo * joy->axes.at(parameters_.axis_rotation_z_id_);
    }
    else
    {
        cmd_vel_.twist.angular.x = 0;
        cmd_vel_.twist.angular.y = 0;
        cmd_vel_.twist.angular.z = 0;
    }

    if (parameters_.have_gripper_ && joy->buttons.at(parameters_.button_translation_deadman_switch_id_) &&
            joy->buttons.at(parameters_.button_rotation_deadman_switch_id_))
    {
        if (joy->buttons.at(parameters_.button_gripper_release_id_))
            sendGripperCommand(parameters_.gripper_release_position_, parameters_.gripper_release_effort_);
        else if (joy->buttons.at(parameters_.button_gripper_grasp_id_))
            sendGripperCommand(parameters_.gripper_grasp_position_, parameters_.gripper_grasp_effort_);
    }

    if (have_vel_commands)
        cmd_vel_.header.stamp = joy->header.stamp;

    if (have_vel_commands || repeat_vel_commands_)
    {
      sendCmdVel();
      repeat_vel_commands_ = have_vel_commands;
    }
}

void SchunkTeleopNode::sendCmdVel()
{
    cmd_vel_pub_.publish(cmd_vel_);
}

void SchunkTeleopNode::sendGripperCommand(double position, double max_effort)
{
    if (gripper_command_ac_)
    {
        control_msgs::GripperCommandGoal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;
        gripper_command_ac_->sendGoal(goal);
    }
}

double SchunkTeleopNode::limit(double value, double min, double max)
{
    if (value < min)
      return min;
    if (value > max)
      return max;
    return value;
}

template <typename T>
void SchunkTeleopNode::getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, T & value)
{
    if (!private_nh.getParam(key, value))
    {
        ROS_FATAL_STREAM("Missing required node parameter " << key);
        throw std::runtime_error("Missing required node parameter " + key);
    }
}

template <typename T>
void SchunkTeleopNode::getOptionalParameter(ros::NodeHandle & private_nh, const std::string & key, T & value, T default_value)
{
    if (!private_nh.getParam(key, value))
    {
        value = default_value;
        ROS_WARN_STREAM("Missing optional node parameter " << key << ", using default value " << default_value);
    }
}




}


int main(int argc, char ** argv)
{
    try
    {
        ros::init(argc, argv, "schunk_teleop_node");
        schunk_teleop::SchunkTeleopNode node;
        node.init();

        ros::spin();
    }
    catch (std::exception & ex)
    {
        ROS_ERROR("Unhandled exception: %s", ex.what());
        return 1;
    }
    catch (...)
    {
        ROS_ERROR("Unhandled exception!");
        return 1;
    }

    return 0;
}
