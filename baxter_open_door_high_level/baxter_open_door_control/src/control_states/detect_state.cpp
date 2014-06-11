#include <baxter_open_door_control/control_states/detect_state.h>
#include <baxter_open_door_control/baxter_open_door_control.h>


namespace baxter_open_door_control
{

DetectState::DetectState(BaxterOpenDoorControl* baxter_open_door_control) :
    BaxterOpenDoorControlState(baxter_open_door_control)
{
}

//******************************************************************************

DetectState::~DetectState()
{

}

//******************************************************************************

bool DetectState::setDetectMode()
{
    ROS_ERROR_STREAM("Set detect mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool DetectState::setGraspHandleMode()
{
    ROS_ERROR_STREAM("Set grasp handle mode from " << getStateName());


    tug_grasp_object_msgs::GraspObjectGoal goal;
    goal.object_to_grasp = baxter_open_door_control_->getObjectToGrasp();
    goal.object_to_grasp.object_pose.header.stamp = ros::Time::now();

    baxter_open_door_control_->getGraspObjectActionClient()->sendGoal(goal);

    return baxter_open_door_control_->setState(baxter_open_door_control_->getGraspHandleState());
}

//******************************************************************************

bool DetectState::setIdleMode()
{
    ROS_ERROR_STREAM("Set idle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool DetectState::setOpenDoorMode()
{
    ROS_ERROR_STREAM("Set open door mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

bool DetectState::setPushHandleMode()
{
    ROS_ERROR_STREAM("Set push handle mode not implemented yet in " << getStateName());
    return false;
}

//******************************************************************************

void DetectState::timedExecution()
{
    baxter_open_door_control_->setFeedback(getStateName(), std::string("idle mode"));

    ROS_ERROR("Detection completed");
    setGraspHandleMode();
}

}
