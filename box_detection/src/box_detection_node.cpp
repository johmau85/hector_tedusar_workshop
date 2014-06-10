#include <box_detection/box_detection_node.h>
#include <boost/bind.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>

/*#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>


#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <box_detection/DetectBoxes.h>*/

static const double BOX_AREA = 0.09 * 0.3;
static const double BOX_AREA_FACTOR = 0.8;
static const double DETECTION_TIMEOUT = 15; // seconds
static const int MIN_POINTS_PER_BOX = 100;


namespace box_detection
{

BoxDetectionNode::BoxDetectionNode()
    : box_detection_as_(nh_, "detect_boxes", false)
{
    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

    pub_cloud_plane = nh_.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);

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
}

void BoxDetectionNode::boxDetectionActionPreemptCallback()
{
    box_detection_as_.setPreempted();
}

void BoxDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    PclPointCloud::Ptr cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr cloud_plane = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr cloud_rest = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr cloud_filtered = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr cloud_transformed = boost::make_shared<PclPointCloud>();

    pcl::fromROSMsg(*msg, *cloud);

    bool box_found = false;
    ros::Duration duration;
    do
    {
        pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
        fitPlane(cloud, inliers);

        if (inliers->indices.size() < MIN_POINTS_PER_BOX)
        {
            break; // not enough points found
        }

        extractPlane(cloud, inliers, *cloud_plane, *cloud_rest);

        downsampleCloud(cloud_plane, *cloud_filtered);

        transformCloud(cloud_filtered, *cloud_transformed);

        // LASTTIME

        std::vector<pcl::PointIndices> cluster_indices;
        clusterizeCloud(cloud_transformed, cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            PclPointCloud cloud_cluster;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            {
                cloud_cluster.points.push_back(cloud_transformed.points[*pit]);
            }

            box_found |= checkClusterForBox(cloud_cluster);
            if (box_found)
            {
                break;  // one found box is enough
            }
        }

        cloud = cloud_rest;
        duration = ros::Time::now() - start_time;
    } while (!box_found && duration.toSec() < DETECTION_TIMEOUT);

    boxes_detected = true;
    sub_pointcloud.shutdown();
}


void BoxDetectionNode::fitPlane(const PclPointCloud::ConstPtr & cloud, pcl::PointIndices::Ptr & inliers)
{
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

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
                                    pcl::PointIndices::ConstPtr & inliers,
                                    PclPointCloud & cloud_plane,
                                    PclPointCloud & cloud_rest)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  extract.setNegative(false);
  extract.filter(cloud_plane);

  extract.setNegative(true);
  extract.filter(cloud_rest);
}

void BoxDetectionNode::downsampleCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_filtered)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(cloud_filtered);
}

void BoxDetectionNode::transformCloud(const PclPointCloud::ConstPtr & cloud, PclPointCloud & cloud_transformed)
{
  sensor_msgs::PointCloud pcin, pcout;
  pcin.header.frame_id = "camera_depth_optical_frame";

  for (PclPointCloud::const_iterator pit = cloud->begin(); pit != cloud->end(); pit++)
  {
    geometry_msgs::Point32 point;
    point.x = pit->x;
    point.y = pit->y;
    point.z = pit->z;
    pcin.points.push_back(point);
  }

  tf_listener_.transformPointCloud("base_link", pcin, pcout);

  for (std::vector<geometry_msgs::Point32>::const_iterator pit = pcout.points.begin(); pit != pcout.points.end(); pit++)
  {
    pcl::PointXYZ point;
    point.x = pit->x;
    point.y = pit->y;
    point.z = pit->z;
    cloud_transformed.push_back(point);
  }
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
ros::Subscriber sub_pointcloud;
ros::Publisher pub_cloud_plane;
ros::Publisher pub_planning_scene_diff;

tf::TransformListener* tf_listener;
tf::StampedTransform tf_transform;

ros::ServiceServer detect_boxes_service;
ros::MultiThreadedSpinner spinner(2);

boost::mutex boxes_detected_mutex;
boost::condition boxes_detected_condition;
bool boxes_detected = false;
std::string box_id;
geometry_msgs::Pose box_pose;


ros::Time start_time;


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



double computeArea(PclPointCloud& cloud,
  pcl::ConvexHull<pcl::PointXYZ>& convex_hull)
{
  PclPointCloud cloud_hull;

  convex_hull.setInputCloud(cloud.makeShared());
  convex_hull.setDimension(2);
  convex_hull.setComputeAreaVolume(true);
  convex_hull.reconstruct(cloud_hull);
  return convex_hull.getTotalArea();
}

void downsampleCloud(PclPointCloud& cloud,
  PclPointCloud& cloud_filtered)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(cloud_filtered);
}

void transformCloud(PclPointCloud& cloud,
  PclPointCloud& cloud_transformed)
{
  sensor_msgs::PointCloud pcin, pcout;
  pcin.header.frame_id = "camera_depth_optical_frame";

  for (PclPointCloud::const_iterator pit = cloud.begin(); pit != cloud.end(); pit++)
  {
    geometry_msgs::Point32 point;
    point.x = pit->x;
    point.y = pit->y;
    point.z = pit->z;

    pcin.points.push_back(point);
  }

  tf_listener->transformPointCloud("base_link", pcin, pcout);

  for (std::vector<geometry_msgs::Point32>::const_iterator pit = pcout.points.begin(); pit != pcout.points.end(); pit++)
  {
    pcl::PointXYZ point;
    point.x = pit->x;
    point.y = pit->y;
    point.z = pit->z;

    cloud_transformed.push_back(point);
  }
}

void clusterizeCloud(PclPointCloud& cloud,
  std::vector<pcl::PointIndices>& cluster_indices)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud.makeShared());

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud(cloud.makeShared());
  ec.setSearchMethod(tree);
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(MIN_POINTS_PER_BOX);
  ec.setMaxClusterSize(cloud.points.size());

  ec.extract(cluster_indices);
}

void publishCollisionObject(pcl::PointXYZ& position, Eigen::Quaternion<float>& orientation)
{
  static unsigned int box_counter = 0;
  moveit_msgs::PlanningScene planning_scene;

  moveit_msgs::CollisionObject object;
  object.header.frame_id = "base_link";

  std::stringstream ss;
  ss << "box" << box_counter++;
  box_id = ss.str();
  object.id = ss.str();

  geometry_msgs::Pose pose;
  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;
  box_pose = pose;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.61;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;
  pub_planning_scene_diff.publish(planning_scene);
}

bool checkClusterForBox(PclPointCloud& cloud)
{
  pcl::ConvexHull<pcl::PointXYZ> convex_hull;
  double area = computeArea(cloud, convex_hull);

  if (area <= BOX_AREA / BOX_AREA_FACTOR && area >= BOX_AREA * BOX_AREA_FACTOR)
  {
    // sensor_msgs::PointCloud2 msg_cloud_plane;
    // convertPointCloudToROSMessage(cloud, msg_cloud_plane);
    // msg_cloud_pane.header.frame_id = "base_link";
    // pub_cloud_plane.publish(msg_cloud_plane);

    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);

    pcl::PointXYZ center;
    center.x = min_pt.x + (max_pt.x - min_pt.x) / 2.0;
    center.y = min_pt.y + (max_pt.y - min_pt.y) / 2.0;
    center.z = min_pt.z + (max_pt.z - min_pt.z) / 2.0;

    // Get major eigenvector:
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud.makeShared());
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f major_vector = eigen_vectors.col(0);
    major_vector.z() = 0;

    Eigen::Quaternion<float> orientation;
    orientation.setFromTwoVectors(Eigen::Vector3f::UnitX(), major_vector);

    publishCollisionObject(center, orientation);

    std::cerr << "[BOX_DETECTION] FOUND BOX!";
    std::cerr << " Area: " << area;
    //std::cerr << " | Clusters: " << cluster_indices.size();
    //std::cerr << " | Points: " << cloud.points.size();
    std::cerr << " | Center: " << center;
    //std::cerr << " | Eigenvector: " << major_vector[0] << " " << major_vector[1] << " " << major_vector[2] << std::endl;
    //std::cerr << " | Angle: " << angle * 180.0/M_PI;
    std::cerr << " | Orientation: " << orientation.w() << " " << orientation.x() << " " << orientation.y() << " " << orientation.z();
    std::cerr << std::endl;
    return true;
  }
  return false;
}



*/
