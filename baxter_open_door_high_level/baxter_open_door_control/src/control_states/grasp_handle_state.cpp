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
    ROS_ERROR_STREAM("Set idle mode from " << getStateName());
    baxter_open_door_control_->getGraspObjectActionClient()->cancelGoal();
    baxter_open_door_control_->getGraspObjectActionClient()->waitForResult(baxter_open_door_control_->getWaitForServerDuration());
    return baxter_open_door_control_->setState(baxter_open_door_control_->getIdleState());
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
    ROS_ERROR_STREAM("Set push handle mode from " << getStateName());
    baxter_open_door_control_->getGraspObjectActionClient()->cancelGoal();
    baxter_open_door_control_->getGraspObjectActionClient()->waitForResult(baxter_open_door_control_->getWaitForServerDuration());

    tug_grasp_object_msgs::PushDoorHandleGoal goal;
    baxter_open_door_control_->getPushHandleActionClient()->sendGoal(goal);

    return  baxter_open_door_control_->setState(baxter_open_door_control_->getPushHandleState());
}

//******************************************************************************

void GraspHandleState::timedExecution()
{
    baxter_open_door_control_->setFeedback(getStateName(), std::string("grasp handle mode"));

    actionlib::SimpleClientGoalState action_state = baxter_open_door_control_->getGraspObjectActionClient()->getState();

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
        setPushHandleMode();
    }
}

}
