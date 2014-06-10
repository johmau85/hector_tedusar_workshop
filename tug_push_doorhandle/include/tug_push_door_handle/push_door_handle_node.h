#ifndef PUSH_DOOR_HANDLE_NODE_
#define PUSH_DOOR_HANDLE_NODE_
#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tug_grasp_object_msgs/PushDoorHandleAction.h>
#include <tug_push_door_handle/push_door_handle/push_door_handle.h>


typedef actionlib::SimpleActionServer<tug_grasp_object_msgs::PushDoorHandleAction> PushDoorHandleActionServer;


namespace push_door_handle_node
{
class PushDoorHandleNode : boost::noncopyable
{
public:
    PushDoorHandleNode();

    virtual ~PushDoorHandleNode();
private:

    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;

    PushDoorHandleActionServer push_door_handle_action_server_;

    push_door_handle::PushDoorHandle push_door_handle_;



    void executePushDoorHandleCB(const tug_grasp_object_msgs::PushDoorHandleGoalConstPtr &goal);



};
}
#endif
