#include <baxter_open_door_control/control_states/open_door_state.h>
#include <baxter_open_door_control/baxter_open_door_control.h>


namespace baxter_open_door_control
{

OpenDoorState::OpenDoorState(BaxterOpenDoorControl* baxter_open_door_control) :
    BaxterOpenDoorControlState(baxter_open_door_control)
{
}

//******************************************************************************

OpenDoorState::~OpenDoorState()
{

}

//******************************************************************************

bool OpenDoorState::setDetectMode()
{
    ROS_ERROR_STREAM("Set detect mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool OpenDoorState::setGraspHandleMode()
{
    ROS_ERROR_STREAM("Set grasp handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool OpenDoorState::setIdleMode()
{
    ROS_ERROR_STREAM("Set idle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool OpenDoorState::setOpenDoorMode()
{
    ROS_ERROR_STREAM("Set open door mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool OpenDoorState::setPushHandleMode()
{
    ROS_ERROR_STREAM("Set push handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

void OpenDoorState::timedExecution()
{
    baxter_open_door_control_->setFeedback(getStateName(), std::string("idle mode"));
}

}
