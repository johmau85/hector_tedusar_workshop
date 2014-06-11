#ifndef OPEN_DOOR_NODE_
#define OPEN_DOOR_NODE_

#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tug_grasp_object_msgs/OpenDoorAction.h>
#include <tug_open_door/open_door/open_door.h>


typedef actionlib::SimpleActionServer<tug_grasp_object_msgs::OpenDoorAction> OpenDoorActionServer;

namespace open_door_node
{
class OpenDoorNode
{

public:
    OpenDoorNode();

    virtual ~OpenDoorNode();
private:

    ros::NodeHandle nh_;

    OpenDoorActionServer open_door_action_server_;


    open_door::OpenDoor open_door_;


    void executeOpenDoorCB(const tug_grasp_object_msgs::OpenDoorGoalConstPtr &goal);

};


}


#endif
