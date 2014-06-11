#ifndef robot_control_h___
#define robot_control_h___

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>

#include <baxter_open_door_control_msgs/BaxterState.h>
#include <baxter_open_door_control_msgs/SetOperationMode.h>

#include <baxter_open_door_control/control_states/baxter_open_door_control_state.h>

#include <tug_grasp_object_msgs/GraspObject.h>

#include <tug_grasp_object_msgs/GraspObjectAction.h>
#include <tug_grasp_object_msgs/OpenDoorAction.h>
#include <tug_grasp_object_msgs/PushDoorHandleAction.h>







typedef boost::shared_ptr<actionlib::SimpleActionClient<tug_grasp_object_msgs::GraspObjectAction> > GraspObjectActionClientPtr;
typedef boost::shared_ptr<actionlib::SimpleActionClient<tug_grasp_object_msgs::OpenDoorAction> > OpenDoorActionClientPtr;
typedef boost::shared_ptr<actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction> > PushDoorHandleActionClientPtr;


namespace baxter_open_door_control
{

class BaxterOpenDoorControl
{
private:
    ros::NodeHandle nh_;

    // robot control feedback publisher
    baxter_open_door_control_msgs::BaxterState baxter_open_door_state_;
    ros::Publisher baxter_open_door_state_pub_;

    // robot control services
//    ros::ServiceServer set_baxter_operation_mode_srv_;


    // robot current state
    BaxterOpenDoorControlStatePtr current_state_;

    // robot states
    BaxterOpenDoorControlStatePtr detect_state_;
    BaxterOpenDoorControlStatePtr grasp_handle_state_;
    BaxterOpenDoorControlStatePtr idle_state_;
    BaxterOpenDoorControlStatePtr open_door_state_;
    BaxterOpenDoorControlStatePtr push_handle_state_;


    // action clients
    PushDoorHandleActionClientPtr push_handle_action_client_;
    GraspObjectActionClientPtr grasp_object_action_client_;
    OpenDoorActionClientPtr open_door_action_client_;

    // timed callback
    ros::Timer timed_function_;


public:
    BaxterOpenDoorControl();

    virtual ~BaxterOpenDoorControl();

    void init();

    // set robot state methode
    void setState(BaxterOpenDoorControlStatePtr state);

    // set feedback for operator
    void setFeedback(std::string state_name, std::string message);


    // get robot states mathodes
    BaxterOpenDoorControlStatePtr getDetectState();
    BaxterOpenDoorControlStatePtr getGraspHandleState();
    BaxterOpenDoorControlStatePtr getIdleState();
    BaxterOpenDoorControlStatePtr getOpenDoorState();
    BaxterOpenDoorControlStatePtr getPushHandleState();


    // get robot states mathodes
    PushDoorHandleActionClientPtr getPushHandleActionClient();
    GraspObjectActionClientPtr getGraspObjectActionClient();
    OpenDoorActionClientPtr getOpenDoorActionClient();

    ros::NodeHandle getNodeHandle()
    {return nh_;}

private:

    // timed callback
    void timedCB(const ros::TimerEvent& e);

};

typedef boost::shared_ptr<BaxterOpenDoorControl> BaxterOpenDoorControlPtr;

}

#endif
