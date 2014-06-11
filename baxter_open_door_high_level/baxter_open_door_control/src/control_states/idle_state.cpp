#include <baxter_open_door_control/control_states/idle_state.h>
#include <baxter_open_door_control/baxter_open_door_control.h>


namespace baxter_open_door_control
{

IdleState::IdleState(BaxterOpenDoorControl* baxter_open_door_control) :
    BaxterOpenDoorControlState(baxter_open_door_control)
{
}

//******************************************************************************

IdleState::~IdleState()
{

}

//******************************************************************************

bool IdleState::setDetectMode()
{
    ROS_ERROR_STREAM("Set detect mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool IdleState::setGraspHandleMode()
{
    ROS_ERROR_STREAM("Set grasp handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool IdleState::setIdleMode()
{
    ROS_ERROR_STREAM("Set idle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool IdleState::setOpenDoorMode()
{
    ROS_ERROR_STREAM("Set open door mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool IdleState::setPushHandleMode()
{
    ROS_ERROR_STREAM("Set push handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

void IdleState::timedExecution()
{
    baxter_open_door_control_->setFeedback(getStateName(), std::string("idle mode"));
}

}
