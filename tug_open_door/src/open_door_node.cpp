#include <tug_open_door/open_door_node.h>

namespace open_door_node
{

OpenDoorNode::OpenDoorNode() :
    nh_(),
    open_door_action_server_(nh_, "open_door", boost::bind( &OpenDoorNode::executeOpenDoorCB,this,_1),false)
{
    ROS_ERROR("OpenDoor Server start");
    open_door_action_server_.start();
}

void OpenDoorNode::executeOpenDoorCB(const tug_grasp_object_msgs::OpenDoorGoalConstPtr &goal)
{
    ROS_ERROR("openening door started");


    while(true)
    {
        if (open_door_action_server_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR("open door execution preempted.");

            open_door_action_server_.setPreempted();

            return;
        }

        if(open_door_.openDoor(goal->midpoint_opening_circle, goal->radius, goal->opening_direction))
        {
             open_door_action_server_.setSucceeded();
             return;
        }
        else
        {
            open_door_action_server_.setAborted();
            return;
        }
    }

}

OpenDoorNode::~OpenDoorNode()
{}


}


int main(int argc, char** argv)
{
//    ROS_ERROR("start main of push handle server");
    ROS_ERROR("main called");
        try
        {
            ros::init(argc,argv,"open_door_node");

            ROS_ERROR("start main of opendoor server");
            open_door_node::OpenDoorNode open_door_node;
            ros::spin();
        }
        catch(...)
        {
            ROS_ERROR("Unhandled exception!");
            return -1;
        }

    return 0;
}
