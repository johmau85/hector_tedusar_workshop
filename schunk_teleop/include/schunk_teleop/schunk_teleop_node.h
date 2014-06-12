#ifndef SCHUNK_TELEOP__SCHUNK_TELEOP_NODE_H_
#define SCHUNK_TELEOP__SCHUNK_TELEOP_NODE_H_

#include <string>
#include <boost/scoped_ptr.hpp>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>


namespace schunk_teleop
{

class SchunkTeleopNode
{
public:
    SchunkTeleopNode();

    void init();

private:
    struct Parameters
    {
        double publication_rate_;
        std::string frame_id_;

        double std_translation_vel_;
        double std_rotation_vel_;

        bool have_gripper_;
        double gripper_grasp_position_;
        double gripper_grasp_effort_;
        double gripper_release_position_;
        double gripper_release_effort_;

        std::string controller_name_;

        int button_translation_deadman_switch_id_;
        int button_rotation_deadman_switch_id_;
        int button_turbo_id_;
        int button_gripper_grasp_id_;
        int button_gripper_release_id_;
        int button_grasp_done_id_;

        int axis_translation_x_id_;
        int axis_translation_y_id_;
        int axis_translation_z_id_;
        int axis_rotation_x_id_;
        int axis_rotation_y_id_;
        int axis_rotation_z_id_;
    };

    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;

    void loadParameters();

    void cmdGeneratorTimerCB(const ros::TimerEvent & e);

    void joyCB(const sensor_msgs::Joy::ConstPtr & joy);

    void sendCmdVel();
    void sendGripperCommand(double position, double max_effort);

    void sendGraspDone();

    template <typename T> static void getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, T & value);
    template <typename T> static void getOptionalParameter(ros::NodeHandle & private_nh, const std::string & key, T & value, T default_value);

    static double limit(double value, double min, double max);

    ros::NodeHandle nh_;
    Parameters parameters_;
    bool repeat_vel_commands_;

    ros::Subscriber joy_sub_;
    ros::Timer cmd_generator_timer_;
    ros::Publisher cmd_vel_pub_;
    boost::scoped_ptr<GripperCommandActionClient> gripper_command_ac_;

    ros::Publisher grasping_done_pub_;

    geometry_msgs::TwistStamped cmd_vel_;
};

}

#endif // SCHUNK_TELEOP__SCHUNK_TELEOP_NODE_H_
