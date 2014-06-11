#include <baxter_open_door_control/control_states/grasp_handle_state.h>
#include <baxter_open_door_control/baxter_open_door_control.h>


namespace baxter_open_door_control
{

GraspHandleState::GraspHandleState(BaxterOpenDoorControl* baxter_open_door_control) :
    BaxterOpenDoorControlState(baxter_open_door_control)
{
}

//******************************************************************************

GraspHandleState::~GraspHandleState()
{

}

//******************************************************************************

bool GraspHandleState::setDetectMode()
{
    ROS_ERROR_STREAM("Set detect mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool GraspHandleState::setGraspHandleMode()
{
    ROS_ERROR_STREAM("Set grasp handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool GraspHandleState::setIdleMode()
{
    ROS_ERROR_STREAM("Set idle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool GraspHandleState::setOpenDoorMode()
{
    ROS_ERROR_STREAM("Set open door mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool GraspHandleState::setPushHandleMode()
{
    ROS_ERROR_STREAM("Set push handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

void GraspHandleState::timedExecution()
{
    baxter_open_door_control_->setFeedback(getStateName(), std::string("idle mode"));
}

}
