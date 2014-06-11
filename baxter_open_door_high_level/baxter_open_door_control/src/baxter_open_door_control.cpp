

#include <baxter_open_door_control/baxter_open_door_control.h>
#include <baxter_open_door_control/control_states/detect_state.h>
#include <baxter_open_door_control/control_states/grasp_handle_state.h>
#include <baxter_open_door_control/control_states/idle_state.h>
#include <baxter_open_door_control/control_states/open_door_state.h>
#include <baxter_open_door_control/control_states/push_handle_state.h>

namespace baxter_open_door_control
{

BaxterOpenDoorControl::BaxterOpenDoorControl() :
    nh_(),
    detect_state_(new DetectState(this)),
    grasp_handle_state_(new GraspHandleState(this)),
    idle_state_(new IdleState(this)),
    open_door_state_(new OpenDoorState(this)),
    push_handle_state_(new PushHandleState(this))
{

}

//******************************************************************************

BaxterOpenDoorControl::~BaxterOpenDoorControl()
{

}

//******************************************************************************

void BaxterOpenDoorControl::init()
{
    ros::NodeHandle private_nh("~");

    // robot control feedback

    baxter_open_door_state_pub_ = nh_.advertise<baxter_open_door_control_msgs::BaxterState>("baxter_open_door_state", 10);


    // robot control services
    set_baxter_operation_mode_srv_ = nh_.advertiseService("set_robot_operation_mode", &BaxterOpenDoorControl::setOperationModeCB, this);



    // action clients
    open_door_action_client_.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::OpenDoorAction>("open_door",false));
    push_handle_action_client_.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction>("push_door_handle",false));
    grasp_object_action_client_.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::GraspObjectAction>("grasp_object",false));

    // timed execution
    double timed_function_hz;
    private_nh.param<double>("timed_function_hz", timed_function_hz, 1.0);
    timed_function_ = nh_.createTimer(ros::Duration(1.0/timed_function_hz), &BaxterOpenDoorControl::timedCB, this);

    //doorhandle in scene
    object_to_grasp_.object_name = "door_handle";
    object_to_grasp_.object_type = "door_handle";

    geometry_msgs::Pose handle_pose;
    handle_pose.orientation.w = -0.233035946881;
    handle_pose.orientation.x = 0.663621432006;
    handle_pose.orientation.y = 0.326102320682 ;
    handle_pose.orientation.z = 0.631631315633;
    handle_pose.position.x = 0.99;
    handle_pose.position.y = -0.36;
    handle_pose.position.z = 0.15;
    object_to_grasp_.object_pose.pose = handle_pose;
    object_to_grasp_.object_pose.header.frame_id = "/base";
    object_to_grasp_.object_pose.header.stamp = ros::Time::now();


    //    door mounting point
    door_mounting_pose_.pose.position.x = 0.65892137268;
    door_mounting_pose_.pose.position.y = -1.06749630216;
    door_mounting_pose_.pose.position.z = 0.312122309527;
    door_mounting_pose_.pose.orientation.w = 1.0;
    door_mounting_pose_.header.frame_id = "base";
    door_mounting_pose_.header.stamp = ros::Time::now();

    door_radius_ = 0.7;
    door_opening_direction_ = "push";



    setState(idle_state_);

}

//******************************************************************************

bool BaxterOpenDoorControl::setState(BaxterOpenDoorControlStatePtr state)
{
    current_state_ = state;
}

//******************************************************************************

void BaxterOpenDoorControl::setFeedback(std::string state, std::string message)
{
    baxter_open_door_state_.state = state;
    baxter_open_door_state_.message = message;
}

//******************************************************************************

BaxterOpenDoorControlStatePtr BaxterOpenDoorControl::getIdleState()
{
    return idle_state_;
}

//******************************************************************************

BaxterOpenDoorControlStatePtr BaxterOpenDoorControl::getDetectState()
{
    return detect_state_;
}
//******************************************************************************

BaxterOpenDoorControlStatePtr BaxterOpenDoorControl::getGraspHandleState()
{
    return grasp_handle_state_;
}

//******************************************************************************

BaxterOpenDoorControlStatePtr BaxterOpenDoorControl::getPushHandleState()
{
    return push_handle_state_;
}


//******************************************************************************

BaxterOpenDoorControlStatePtr BaxterOpenDoorControl::getOpenDoorState()
{
    return open_door_state_;
}


//******************************************************************************

GraspObjectActionClientPtr BaxterOpenDoorControl::getGraspObjectActionClient()
{
    return grasp_object_action_client_;
}

//******************************************************************************

PushDoorHandleActionClientPtr BaxterOpenDoorControl::getPushHandleActionClient()
{
    return push_handle_action_client_;
}

//******************************************************************************

OpenDoorActionClientPtr BaxterOpenDoorControl::getOpenDoorActionClient()
{
    return open_door_action_client_;
}


//******************************************************************************

bool BaxterOpenDoorControl::setOperationModeCB(baxter_open_door_control_msgs::SetOperationMode::Request &request, baxter_open_door_control_msgs::SetOperationMode::Response &response)
{
    bool result = false;

    switch(request.operation_mode)
    {
    case baxter_open_door_control_msgs::SetOperationMode::Request::DETECT_MODE:
        result = current_state_->setDetectMode();
        break;
    case baxter_open_door_control_msgs::SetOperationMode::Request::GRASP_HANDLE_MODE:
        result = current_state_->setGraspHandleMode();
        break;
    case baxter_open_door_control_msgs::SetOperationMode::Request::IDLE_MODE:
        result = current_state_->setIdleMode();
        break;
    case baxter_open_door_control_msgs::SetOperationMode::Request::OPEN_DOOR_MODE:
        result = current_state_->setOpenDoorMode();
        break;
    case baxter_open_door_control_msgs::SetOperationMode::Request::PUSH_HANDLE_MODE:
        result = current_state_->setPushHandleMode();
        break;
    default:
        ROS_WARN("Unknown operation mode.");
    }

    return result;

}

//******************************************************************************

void BaxterOpenDoorControl::timedCB(const ros::TimerEvent& e)
{
    // do computetions of current states
    current_state_->timedExecution();

    // publish robot state
    baxter_open_door_state_pub_.publish(baxter_open_door_state_);
}


} // end namespace robot_control
