#ifndef GRASP_OBJECT_NODE
#define GRASP_OBJECT_NODE
#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tug_grasp_object_msgs/GraspObjectAction.h>
#include <tug_grasp_object/grasp_object/grasp_object.h>

typedef actionlib::SimpleActionServer<tug_grasp_object_msgs::GraspObjectAction> GraspObjectActionServer;


namespace grasp_object_node
{
class GraspObjectNode : boost::noncopyable
{
public:
    GraspObjectNode();

    virtual ~GraspObjectNode();
private:

    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;

    GraspObjectActionServer grasp_object_action_server_;
    ros::Duration wait_for_server_duration_;
    double control_rate_;




    void executeGraspObjectCB(const tug_grasp_object_msgs::GraspObjectGoalConstPtr &goal);



};
}



#endif
