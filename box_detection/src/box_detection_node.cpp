#include <box_detection/box_detection_node.h>
#include <sstream>
#include <boost/bind.hpp>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

/*#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <box_detection/DetectBoxes.h>*/

static const double BOX_AREA = 0.09 * 0.3;
static const double BOX_AREA_FACTOR = 0.8;
static const ros::Duration DETECTION_TIMEOUT(15.0);
static const int MIN_POINTS_PER_BOX = 100;


namespace box_detection
{

BoxDetectionNode::BoxDetectionNode()
    : box_detection_as_(nh_, "detect_boxes", false), box_counter_(0)
{
    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

    plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
    planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    box_detection_as_.registerGoalCallback(boost::bind(&BoxDetectionNode::boxDetectionActionGoalCallback, this));
    box_detection_as_.registerPreemptCallback(boost::bind(&BoxDetectionNode::boxDetectionActionPreemptCallback, this));
    box_detection_as_.start();
}

void BoxDetectionNode::boxDetectionActionGoalCallback()
{
    box_detection_as_.acceptNewGoal();

    try
    {
      tf_listener_.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time::now(), ros::Duration(10.0));
    }
    catch (tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("TransformException: " << ex.what());
      box_detection_as_.setAborted(BoxDetectionResult(), "transformation failed");
      return;
    }

    // Subscribe to pointcloud:
    point_cloud_subscriber_ = nh_.subscribe("/camera/depth/points", 1, &BoxDetectionNode::pointCloudCallback, this);
    start_time_ = ros::Time::now();
    boxes_.clear();
}

void BoxDetectionNode::boxDetectionActionPreemptCallback()
{
    box_detection_as_.setPreempted();
}

void BoxDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    point_cloud_subscriber_.shutdown();

    PclPointCloud::Ptr cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr plane_cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr remaining_cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr downsampled_plane_cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr transformed_plane_cloud = boost::make_shared<PclPointCloud>();

    pcl::fromROSMsg(*msg, *cloud);

    bool box_found = false;
    do
    {
        pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
        fitPlane(cloud, inliers);

        if (inliers->indices.size() < MIN_POINTS_PER_BOX)
            break; // not enough points found

        extractPlane(cloud, inliers, *plane_cloud, *remaining_cloud);

        downsampleCloud(plane_cloud, *downsampled_plane_cloud);

        transformCloud(msg->header.frame_id, "base_link", downsampled_plane_cloud, *transformed_plane_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        clusterizeCloud(transformed_plane_cloud, cluster_indices);

        pcl::ExtractIndices<PclPoint> extractor;
        pcl::PointIndices::Ptr point_indices = boost::make_shared<pcl::PointIndices>();
        extractor.setInputCloud(transformed_plane_cloud);
        extractor.setIndices(point_indices);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            *point_indices = *it;
            PclPointCloud::Ptr cloud_cluster = boost::make_shared<PclPointCloud>();
            extractor.filter(*cloud_cluster);

            if (checkClusterForBox(cloud_cluster))
            {
                const Box & box = createBox(cloud_cluster);
                publishBoxCloud(box, cloud_cluster);
                publishCollisionObject(box);
                publishActionFeedback(box);

                ROS_INFO_STREAM("FOUND BOX! Name: " << box.name_ << "; Area: " << computeArea(cloud_cluster)  << "; Pose: " << box.pose_);
                box_found = true;
            }


        }

        cloud = remaining_cloud;
    }
    while (!box_found && box_detection_as_.isActive() && (ros::Time::now() - start_time_) < DETECTION_TIMEOUT);

    if (box_detection_as_.isActive())
    {
        BoxDetectionResult result;
        for (std::vector<Box>::iterator box_it = boxes_.begin(); box_it != boxes_.end(); ++box_it)
        {
            result.box_names.push_back(box_it->name_);
            result.box_poses.push_back(box_it->pose_);
        }
        box_detection_as_.setSucceeded(result);
    }
}


void BoxDetectionNode::fitPlane(const PclPointCloud::ConstPtr & cloud, pcl::PointIndices::Ptr & inliers)
{
    pcl::SACSegmentation<PclPoint> segmentation;

    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(0.15);
    segmentation.setMaxIterations(50);
    segmentation.setDistanceThreshold(0.02);
    segmentation.setInputCloud(cloud);

    //seg.setOptimizeCoefficients(true);

    pcl::ModelCoefficients coefficients;
    segmentation.segment(*inliers, coefficients);
}

void BoxDetectionNode::extractPlane(const PclPointCloud::ConstPtr & cloud,
                                    const pcl::PointIndices::ConstPtr & inliers,
                                    PclPointCloud & cloud_plane,
                                    PclPointCloud & cloud_rest)
{
    pcl::ExtractIndices<PclPoint> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(cloud_plane);

    extract.setNegative(true);
    extract.filter(cloud_rest);
}

void BoxDetectionNode::downsampleCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_filtered)
{
    pcl::VoxelGrid<PclPoint> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.01, 0.01, 0.01);
    filter.filter(cloud_filtered);
}

void BoxDetectionNode::transformCloud(const std::string & source_frame_id, const std::string & target_frame_id,
                                      const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_transformed)
{
    sensor_msgs::PointCloud pcin, pcout;
    pcin.header.frame_id = source_frame_id;

    for (PclPointCloud::const_iterator pit = cloud->begin(); pit != cloud->end(); pit++)
    {
        geometry_msgs::Point32 point;
        point.x = pit->x;
        point.y = pit->y;
        point.z = pit->z;
        pcin.points.push_back(point);
    }

    tf_listener_.transformPointCloud(target_frame_id, pcin, pcout);

    for (std::vector<geometry_msgs::Point32>::const_iterator pit = pcout.points.begin(); pit != pcout.points.end(); pit++)
    {
        PclPoint point;
        point.x = pit->x;
        point.y = pit->y;
        point.z = pit->z;
        cloud_transformed.push_back(point);
    }
}

void BoxDetectionNode::clusterizeCloud(const PclPointCloud::ConstPtr & cloud, std::vector<pcl::PointIndices> & cluster_indices)
{
    pcl::search::KdTree<PclPoint>::Ptr tree = boost::make_shared<pcl::search::KdTree<PclPoint> >();
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PclPoint> ece;
    ece.setInputCloud(cloud);
    ece.setSearchMethod(tree);
    ece.setClusterTolerance(0.02);
    ece.setMinClusterSize(MIN_POINTS_PER_BOX);
    ece.setMaxClusterSize(cloud->size());

    ece.extract(cluster_indices);
}

bool BoxDetectionNode::checkClusterForBox(const PclPointCloud::ConstPtr & cloud)
{
    double area = computeArea(cloud);
    return BOX_AREA * BOX_AREA_FACTOR <= area && area <= BOX_AREA / BOX_AREA_FACTOR;
}

const BoxDetectionNode::Box & BoxDetectionNode::createBox(const PclPointCloud::ConstPtr & cloud)
{
    PclPoint min_pt;
    PclPoint max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    PclPoint center;
    center.x = (min_pt.x + max_pt.x) / 2.0f;
    center.y = (min_pt.y + max_pt.y) / 2.0f;
    center.z = (min_pt.z + max_pt.z) / 2.0f;

    // Get major eigenvector:
    pcl::PCA<PclPoint> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f major_vector = eigen_vectors.col(0);
    major_vector.z() = 0;

    Eigen::Quaternion<float> orientation;
    orientation.setFromTwoVectors(Eigen::Vector3f::UnitX(), major_vector);

    Box box;

    std::ostringstream ss;
    ss << "box" << box_counter_++;
    box.name_ = ss.str();

    box.pose_.header.frame_id = "base_link";
    box.pose_.header.stamp = ros::Time::now();
    box.pose_.pose.orientation.x = orientation.x();
    box.pose_.pose.orientation.y = orientation.y();
    box.pose_.pose.orientation.z = orientation.z();
    box.pose_.pose.orientation.w = orientation.w();
    box.pose_.pose.position.x = center.x;
    box.pose_.pose.position.y = center.y;
    box.pose_.pose.position.z = center.z;

    boxes_.push_back(box);

    return boxes_.back();
}

void BoxDetectionNode::publishBoxCloud(const Box & box, const PclPointCloud::ConstPtr & cloud)
{
    sensor_msgs::PointCloud2 plane_cloud_msg;
    pcl::toROSMsg(*cloud, plane_cloud_msg);
    plane_cloud_msg.header = box.pose_.header;
    plane_cloud_publisher_.publish(plane_cloud_msg);
}

void BoxDetectionNode::publishCollisionObject(const Box & box)
{
    moveit_msgs::CollisionObject object;
    object.header = box.pose_.header;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.61;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    object.id = box.name_;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box.pose_.pose);

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene.is_diff = true;
    planning_scene_publisher_.publish(planning_scene);
}

void BoxDetectionNode::publishActionFeedback(const Box & box)
{
    BoxDetectionFeedback feedback;
    feedback.box_name = box.name_;
    feedback.box_pose = box.pose_;
    box_detection_as_.publishFeedback(feedback);
}

double BoxDetectionNode::computeArea(const PclPointCloud::ConstPtr & cloud)
{
    pcl::ConvexHull<PclPoint> convex_hull;
    PclPointCloud cloud_hull;
    convex_hull.setInputCloud(cloud);
    convex_hull.setDimension(2);
    convex_hull.setComputeAreaVolume(true);
    convex_hull.reconstruct(cloud_hull);
    return convex_hull.getTotalArea();
}




}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "box_detection");

    try
    {
        box_detection::BoxDetectionNode node;
        ros::spin();
        return 0;
    }
    catch (std::exception & ex)
    {
        ROS_FATAL("Unhandled exception: %s", ex.what());
        return 1;
    }

    //pub_planning_scene_diff = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    // while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    // {
    //   ros::WallDuration sleep_t(0.5);
    //   sleep_t.sleep();
    // }

    // Create DetectBoxes service:
    //detect_boxes_service = pnh.advertiseService(
    //            box_detection::DetectBoxes::Request::SERVICE_NAME, detectBoxes);

    //spinner.spin();
}




















/*
ros::Publisher pub_cloud_plane;

ros::MultiThreadedSpinner spinner(2);


void convertROSMessageToPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg,
  PclPointCloud& cloud)
{
  #if PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 7
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
  #else
    pcl::fromROSMsg(*msg, cloud);
  #endif
}

void convertPointCloudToROSMessage(PclPointCloud& cloud,
  sensor_msgs::PointCloud2& msg)
{
  #if PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 7
    pcl::toROSMsg(cloud, msg);
  #else
    pcl::toROSMsg(cloud, msg);
  #endif
}



*/
