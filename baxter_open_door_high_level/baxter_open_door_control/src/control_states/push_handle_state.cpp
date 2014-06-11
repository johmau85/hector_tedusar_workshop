#include <baxter_open_door_control/control_states/push_handle_state.h>
#include <baxter_open_door_control/baxter_open_door_control.h>


namespace baxter_open_door_control
{

PushHandleState::PushHandleState(BaxterOpenDoorControl* baxter_open_door_control) :
    BaxterOpenDoorControlState(baxter_open_door_control)
{
}

//******************************************************************************

PushHandleState::~PushHandleState()
{

}

//******************************************************************************

bool PushHandleState::setDetectMode()
{
    ROS_ERROR_STREAM("Set detect mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool PushHandleState::setGraspHandleMode()
{
    ROS_ERROR_STREAM("Set grasp handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool PushHandleState::setIdleMode()
{
    ROS_ERROR_STREAM("Set idle mode from " << getStateName());
    baxter_open_door_control_->getPushHandleActionClient()->cancelGoal();
    baxter_open_door_control_->getPushHandleActionClient()->waitForResult(baxter_open_door_control_->getWaitForServerDuration());
    return baxter_open_door_control_->setState(baxter_open_door_control_->getIdleState());
}

//******************************************************************************

bool PushHandleState::setOpenDoorMode()
{
    ROS_ERROR_STREAM("Set open door mode from " << getStateName());
    baxter_open_door_control_->getPushHandleActionClient()->cancelGoal();
    baxter_open_door_control_->getPushHandleActionClient()->waitForResult(baxter_open_door_control_->getWaitForServerDuration());

    tug_grasp_object_msgs::OpenDoorGoal goal;
    goal.midpoint_opening_circle = baxter_open_door_control_->getDoorMountingPose();
    goal.opening_direction = baxter_open_door_control_->getDoorOpeningDirection();
    goal.radius = baxter_open_door_control_->getDoorRadius();
    return baxter_open_door_control_->setState(baxter_open_door_control_->getOpenDoorState());
}

//******************************************************************************

bool PushHandleState::setPushHandleMode()
{
    ROS_ERROR_STREAM("Set push handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

void PushHandleState::timedExecution()
{
    baxter_open_door_control_->setFeedback(getStateName(), std::string("push handle state"));

    actionlib::SimpleClientGoalState action_state = baxter_open_door_control_->getPushHandleActionClient()->getState();

    if(!((action_state == actionlib::SimpleClientGoalState::PENDING) || (action_state == actionlib::SimpleClientGoalState::ACTIVE)  || (action_state == actionlib::SimpleClientGoalState::SUCCEEDED)))
    {
        if((action_state == actionlib::SimpleClientGoalState::RECALLED) || (action_state == actionlib::SimpleClientGoalState::PREEMPTED))
        {
            ROS_WARN_STREAM("The action was canceled during state: " << getStateName());
            setIdleMode();
        }
        else if(action_state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_WARN_STREAM("The action reported an execution error during state: " << getStateName());
            setIdleMode();
        }
        else if(action_state == actionlib::SimpleClientGoalState::REJECTED)
        {
            ROS_WARN_STREAM("The action was rejected in state: " << getStateName());
            setIdleMode();
        }
        else
        {
            ROS_WARN_STREAM("Unhandled action state in state: " << getStateName());
            setIdleMode();
        }
    }

    if(action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("Action sucessfull complited during state: " << getStateName());
        setOpenDoorMode();
    }

}

}
