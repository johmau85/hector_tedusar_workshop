#ifndef BOX_DETECTION_AND_SHORING__SHORING_NODE_H_
#define BOX_DETECTION_AND_SHORING__SHORING_NODE_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <actionlib/client/simple_action_client.h>
#include <shoring_msgs/PlaceBlockAction.h>
#include <box_detection/BoxDetectionAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>

namespace box_detection_and_shoring
{

class ShoringNode
{
public:
    ShoringNode();
    void run();

private:
    bool detectBox(geometry_msgs::PoseStamped & box_pose);
    bool switchControllers(bool switch_to_cartesian);
    bool placeBlock();
    bool waitUntilGraspingDone();

    void graspingDoneCallback(const std_msgs::EmptyConstPtr &);

    bool moveToPose(const geometry_msgs::PoseStamped & pose);
    void moveArmCartesian(const geometry_msgs::PoseStamped & point_1, const geometry_msgs::PoseStamped & point_2);

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<shoring_msgs::PlaceBlockAction> place_block_action_client_;
    actionlib::SimpleActionClient<box_detection::BoxDetectionAction> box_detection_action_client_;
    ros::ServiceClient switch_controller_service_client_;
    ros::Subscriber grasping_done_subscriber_;
    bool grasping_done_;

    tf::TransformListener tf_listener_;
    boost::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;
    std::string planning_frame_id_;
    std::string heap_frame_id_;
    geometry_msgs::PoseStamped scan_pose_;
};

}

#endif // BOX_DETECTION_AND_SHORING__SHORING_NODE_H_
