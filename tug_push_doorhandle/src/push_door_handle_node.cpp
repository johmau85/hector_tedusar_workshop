#include<tug_push_door_handle/push_door_handle_node.h>

namespace push_door_handle_node
{

PushDoorHandleNode::PushDoorHandleNode() :
    nh_(),
    tf_listener_(),
    push_door_handle_action_server_(nh_, "push_door_handle", boost::bind( &PushDoorHandleNode::executePushDoorHandleCB,this,_1),false)
{
    ROS_ERROR("Server start");
    push_door_handle_action_server_.start();
}

void PushDoorHandleNode::executePushDoorHandleCB(const tug_grasp_object_msgs::PushDoorHandleGoalConstPtr &goal)
{
    ROS_ERROR("pushing handle started");
    tug_grasp_object_msgs::PushDoorHandleResult push_door_handle_result;


    while(true)
    {
        ROS_ERROR_STREAM("TEST true");
        if (push_door_handle_action_server_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR("Push door handle execution preempted.");

            push_door_handle_result.result = tug_grasp_object_msgs::PushDoorHandleResult::EXECUTION_PREEMPTED;
            push_door_handle_action_server_.setPreempted(push_door_handle_result, std::string("Pushing door handle action preempted"));

            return;
        }

        if(push_door_handle_.pushHandle())
        {
             push_door_handle_action_server_.setSucceeded();
             return;
        }
        else
        {
            push_door_handle_action_server_.setAborted();
            return;
        }
    }

}

PushDoorHandleNode::~PushDoorHandleNode()
{}


}

int main(int argc, char** argv)
{
//    ROS_ERROR("start main of push handle server");
        try
        {
            ros::init(argc,argv,"push_door_handle_node");

            ROS_ERROR("start main of push handle server");
            push_door_handle_node::PushDoorHandleNode node;

            ros::spin();
        }
        catch(...)
        {
            ROS_ERROR("Unhandled exception!");
            return -1;
        }

    return 0;
}
