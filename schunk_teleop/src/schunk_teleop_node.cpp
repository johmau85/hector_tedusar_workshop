#include <schunk_teleop/schunk_teleop_node.h>
#include <stdexcept>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

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

    cmd_generator_timer_ = nh_.createTimer(ros::Duration(1.0 / hz),
        &SchunkTeleopNode::cmdGeneratorTimerCB, this, false);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cartesian_controller/vel_cmd", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
        &SchunkTeleopNode::joyCB, this);
}

void SchunkTeleopNode::loadParameters()
{
    ros::NodeHandle private_nh("~");

    double hz;
    private_nh.param<double>("hz", hz, 20.0);

    private_nh.param<double>("std_trans_vel", parameters_.std_trans_vel_, 0.2);
    private_nh.param<double>("std_rot_vel", parameters_.std_rot_vel_, 0.2);

    private_nh.param<std::string>("controller", parameters_.controller_name_, "No config default");
    getRequiredParameter(private_nh, "button_move_deadman_switch", parameters_.button_move_deadman_switch_id_);
    getRequiredParameter(private_nh, "button_rotate_deadman_switch", parameters_.button_rotate_deadman_switch_id_);
    getRequiredParameter(private_nh, "button_turbo", parameters_.button_turbo_id_);

    getRequiredParameter(private_nh, "axis_move_x", parameters_.axis_move_x_id_);
    getRequiredParameter(private_nh, "axis_move_y", parameters_.axis_move_y_id_);
    getRequiredParameter(private_nh, "axis_move_z", parameters_.axis_move_z_id_);
    getRequiredParameter(private_nh, "axis_rotate_x", parameters_.axis_rotate_x_id_);
    getRequiredParameter(private_nh, "axis_rotate_y", parameters_.axis_rotate_y_id_);
    getRequiredParameter(private_nh, "axis_rotate_z", parameters_.axis_rotate_z_id_);
}

void SchunkTeleopNode::cmdGeneratorTimerCB(const ros::TimerEvent& e)
{
    if (repeat_vel_commands_)
        sendCmdVel();
}

void SchunkTeleopNode::joyCB(const sensor_msgs::Joy::ConstPtr& joy)
{
    bool have_vel_commands = false;

    double turbo = joy->buttons[parameters_.button_turbo_id_] ? 2 : 1;

    if (joy->buttons[parameters_.button_move_deadman_switch_id_] && !joy->buttons[parameters_.button_rotate_deadman_switch_id_])
    {
      have_vel_commands = true;
      cmd_vel_.linear.x = std_trans_vel_ * joy->axes[axis_base_move_id_]
          * (joy->buttons[button_base_turbo_id_] ? 2 : 1);
      cmd_vel_.angular.z = std_rot_vel_ * joy->axes[axis_base_rotate_id_]
          * (joy->buttons[button_base_turbo_id_] ? 2 : 1);
    }
    else
    {
      cmd_vel_.linear.x = 0;
      cmd_vel_.angular.z = 0;
    }

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

static double SchunkTeleopNode::limit(double value, double min, double max)
{
    if (value < min)
      return min;
    if (value > max)
      return max;
    return value;
}

void SchunkTeleopNode::getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, int & value)
{
    if (!private_nh.getParam(key, value))
        throw std::runtime_error("Missing node parameter " + key);
}




}


int main(int argc, char** argv)
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
