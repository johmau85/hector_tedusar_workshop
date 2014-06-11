#ifndef BOX_DETECTION__BOX_DETECTION_H_
#define BOX_DETECTION__BOX_DETECTION_H_

// Prevent unaligned array exception:
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/PlanningScene.h>
#include <box_detection/BoxDetectionAction.h>

namespace box_detection
{

class BoxDetectionNode
{
public:
    BoxDetectionNode();

private:
    typedef pcl::PointXYZ PclPoint;
    typedef pcl::PointCloud<PclPoint> PclPointCloud;

    struct Box
    {
        std::string name_;
        geometry_msgs::PoseStamped pose_;
    };

    void boxDetectionActionGoalCallback();
    void boxDetectionActionPreemptCallback();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
    void publisherTimerCallback(const ros::TimerEvent &);

    void fitPlane(const PclPointCloud::ConstPtr & cloud, pcl::PointIndices::Ptr & inliers);
    void extractPlane(const PclPointCloud::ConstPtr & cloud,
                      const pcl::PointIndices::ConstPtr & inliers,
                      PclPointCloud & cloud_plane,
                      PclPointCloud & cloud_rest);
    void downsampleCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_filtered);
    void transformCloud(const std::string & source_frame_id, const std::string & target_frame_id,
                        const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_transformed);
    void clusterizeCloud(const PclPointCloud::ConstPtr & cloud, std::vector<pcl::PointIndices> & cluster_indices);
    bool checkClusterForBox(const PclPointCloud::ConstPtr & cloud);
    const Box & createBox(const PclPointCloud::ConstPtr & cloud);
    void publishBoxCloud(const Box & box, const PclPointCloud::ConstPtr & cloud);
    double computeArea(const PclPointCloud::ConstPtr & cloud);
    void publishCollisionObject(const Box & box);
    void publishActionFeedback(const Box & box);

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<box_detection::BoxDetectionAction> box_detection_as_;
    tf::TransformListener tf_listener_;
    ros::Subscriber point_cloud_subscriber_;
    ros::Publisher plane_cloud_publisher_;
    ros::Timer publisher_timer_;
    ros::Publisher planning_scene_publisher_;
    ros::Time start_time_;
    std::vector<Box> boxes_;
    sensor_msgs::PointCloud plane_cloud_;
    moveit_msgs::PlanningScene planning_scene_;
    unsigned int box_counter_;
};

}

#endif // BOX_DETECTION__BOX_DETECTION_H_
