#ifndef BOX_DETECTION__BOX_DETECTION_H_
#define BOX_DETECTION__BOX_DETECTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <box_detection/BoxDetectionAction.h>

namespace box_detection
{

class BoxDetectionNode
{
public:
    BoxDetectionNode();

private:
    typedef pcl::PointCloud<pcl::PointXYZ> PclPointCloud;

    void boxDetectionActionGoalCallback();
    void boxDetectionActionPreemptCallback();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg);

    void fitPlane(const PclPointCloud::ConstPtr & cloud, pcl::PointIndices::Ptr & inliers);
    void extractPlane(const PclPointCloud::ConstPtr & cloud,
                      pcl::PointIndices::ConstPtr & inliers,
                      PclPointCloud & cloud_plane,
                      PclPointCloud & cloud_rest);
    void downsampleCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_filtered);
    void transformCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_transformed);

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<box_detection::BoxDetectionAction> box_detection_as_;
    tf::TransformListener tf_listener_;
    ros::Publisher pub_cloud_plane;
    ros::Subscriber point_cloud_subscriber_;
    ros::Time start_time_;
};

}

#endif // BOX_DETECTION__BOX_DETECTION_H_
