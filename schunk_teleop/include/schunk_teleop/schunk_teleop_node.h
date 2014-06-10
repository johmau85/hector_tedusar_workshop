#ifndef SCHUNK_TELEOP__SCHUNK_TELEOP_NODE_H_
#define SCHUNK_TELEOP__SCHUNK_TELEOP_NODE_H_

#include <ros/node_handle.h>

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
        double std_trans_vel_;
        double std_rot_vel_;
        std::string controller_name_;

        int button_move_deadman_switch_id_;
        int button_rotate_deadman_switch_id_;
        int button_turbo_id_;

        int axis_move_x_id_;
        int axis_move_y_id_;
        int axis_move_z_id_;
        int axis_rotate_x_id_;
        int axis_rotate_y_id_;
        int axis_rotate_z_id_;
    };

    void loadParameters();

    void cmdGeneratorTimerCB(const ros::TimerEvent& e);

    void joyCB(const sensor_msgs::Joy::ConstPtr& joy);

    void sendCmdVel();

    void getRequiredParameter(ros::NodeHandle & private_nh, const std::string & key, int & value);

    static double limit(double value, double min, double max);

    ros::NodeHandle nh_;
    Parameters parameters_;
    bool repeat_vel_commands_;

    ros::Subscriber joy_sub_;
    ros::Timer cmd_generator_timer_;
    ros::Publisher cmd_vel_pub_;

    geometry_msgs::Twist cmd_vel_;
};

}

#endif // SCHUNK_TELEOP__SCHUNK_TELEOP_NODE_H_
