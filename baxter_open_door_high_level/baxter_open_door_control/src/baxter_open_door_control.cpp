

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
    //set_baxter_operation_mode_srv_ = nh_.advertiseService("set_robot_operation_mode", &RobotControl::setRobotOperationModeCB, this);



    // action clients
    open_door_action_client_.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::OpenDoorAction>("open_door",false));
    push_handle_action_client_.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction>("push_door_handle",false));
    grasp_object_action_client_.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::GraspObjectAction>("grasp_object",false));

    // timed execution
    double timed_function_hz;
    private_nh.param<double>("timed_function_hz", timed_function_hz, 1.0);
    timed_function_ = nh_.createTimer(ros::Duration(1.0/timed_function_hz), &BaxterOpenDoorControl::timedCB, this);

    setState(idle_state_);

}

//******************************************************************************

void BaxterOpenDoorControl::setState(BaxterOpenDoorControlStatePtr state)
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

//bool RobotControl::setRobotOperationModeCB(SetRobotOperationMode::Request &request, SetRobotOperationMode::Response &response)
//{
//    bool result = false;

//    switch(request.operation_mode)
//    {
//    case SetRobotOperationMode::Request::AUTONOMOUS_MODE:
//        result = current_state_->setExploreAndSearchMode();
//        break;
//    case SetRobotOperationMode::Request::TELEOP_MODE:
//        result = current_state_->setTeleopMode();
//        break;
//    case SetRobotOperationMode::Request::LOOK_AROUND_MODE:
//        result = current_state_->setLookAroundMode();
//        break;
//    case SetRobotOperationMode::Request::EXPLORATION_MODE:
//        result = current_state_->setExplorationMode();
//        break;
//    case SetRobotOperationMode::Request::DETECTOR_CHECK_MODE:
//        result = current_state_->setDetectorCheckMode();
//        break;
//    case SetRobotOperationMode::Request::GO_TO_OBSERVE_POSE_MODE:
//        result = current_state_->setGoToObservePoseMode();
//        break;
//    case SetRobotOperationMode::Request::OBSERVE_MODE:
//        result = current_state_->setObserveMode();
//        break;
//    case SetRobotOperationMode::Request::VERIFY_VICTIM_MODE:
//        result = current_state_->setVerifyVictimMode();
//        break;
//    default:
//        ROS_WARN("Unknown operation mode.");
//    }

//    return result;
//}

//******************************************************************************

void BaxterOpenDoorControl::timedCB(const ros::TimerEvent& e)
{
    // do computetions of current states
    current_state_->timedExecution();

    // publish robot state
    baxter_open_door_state_pub_.publish(baxter_open_door_state_);
}


} // end namespace robot_control
