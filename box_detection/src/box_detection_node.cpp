#include <box_detection/box_detection_node.h>
#include <iostream>
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
#include <moveit_msgs/CollisionObject.h>



static const double BOX_AREA = 0.09 * 0.3;
static const double BOX_AREA_FACTOR = 0.8;
static const ros::Duration DETECTION_TIMEOUT(15.0);
static const int MIN_POINTS_PER_BOX = 50;


static pcl::PointXYZ operator+(const pcl::PointXYZ & p1, const pcl::PointXYZ & p2)
{
    return pcl::PointXYZ(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

static pcl::PointXYZ operator-(const pcl::PointXYZ & p1, const pcl::PointXYZ & p2)
{
    return pcl::PointXYZ(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

static pcl::PointXYZ operator/(const pcl::PointXYZ & p, float f)
{
    return pcl::PointXYZ(p.x / f, p.y / f, p.z / f);
}



namespace box_detection
{

BoxDetectionNode::BoxDetectionNode()
    : box_detection_as_(nh_, "detect_boxes", false), box_counter_(0)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud>("plane_cloud", 1);
    publisher_timer_ = nh_.createTimer(ros::Rate(2.0), &BoxDetectionNode::publisherTimerCallback, this);

    box_detection_as_.registerGoalCallback(boost::bind(&BoxDetectionNode::boxDetectionActionGoalCallback, this));
    box_detection_as_.registerPreemptCallback(boost::bind(&BoxDetectionNode::boxDetectionActionPreemptCallback, this));
    box_detection_as_.start();
}

void BoxDetectionNode::boxDetectionActionGoalCallback()
{
    ROS_INFO("BoxDetection action started");
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

    plane_cloud_ = sensor_msgs::PointCloud();
    plane_cloud_.header.frame_id = "base_link";
    plane_cloud_.channels.resize(1);
    plane_cloud_.channels.at(0).name = "intensity";

    boxes_.clear();
}

void BoxDetectionNode::boxDetectionActionPreemptCallback()
{
    box_detection_as_.setPreempted();
    ROS_INFO("BoxDetection action preempted");
}

void BoxDetectionNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    point_cloud_subscriber_.shutdown();

    PclPointCloud::Ptr cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr plane_cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr remaining_cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr downsampled_plane_cloud = boost::make_shared<PclPointCloud>();
    PclPointCloud::Ptr transformed_cloud = boost::make_shared<PclPointCloud>();

    pcl::fromROSMsg(*msg, *cloud);
    transformCloud(msg->header.frame_id, "base_link", cloud, *transformed_cloud);

    bool box_found = false;
    float plane_intensity = 0;
    do
    {
        pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
        fitPlane(transformed_cloud, inliers);

        if (inliers->indices.size() < MIN_POINTS_PER_BOX)
            break; // not enough points found

        ROS_INFO("Found plane with %d points", inliers->indices.size());

        extractPlane(transformed_cloud, inliers, *plane_cloud, *remaining_cloud);
        downsampleCloud(plane_cloud, *downsampled_plane_cloud);

        ROS_INFO("Downsampled it to %d points", downsampled_plane_cloud->size());

        std::vector<pcl::PointIndices> cluster_indices;
        clusterizeCloud(downsampled_plane_cloud, cluster_indices);



        ROS_INFO("Clustered into %d clusters", cluster_indices.size());

        pcl::ExtractIndices<PclPoint> extractor;
        pcl::PointIndices::Ptr point_indices = boost::make_shared<pcl::PointIndices>();
        extractor.setInputCloud(downsampled_plane_cloud);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            *point_indices = *it;
            extractor.setIndices(point_indices); // Must be done after assigning new indices to point_indices!
            PclPointCloud::Ptr cloud_cluster = boost::make_shared<PclPointCloud>();
            extractor.filter(*cloud_cluster);
            ROS_INFO("Checking cluster with %d points", cloud_cluster->size());

            // Add cluster to plane cloud:
            plane_intensity += 1.0f;
            for (PclPointCloud::iterator p_it = cloud_cluster->begin(); p_it != cloud_cluster->end(); ++p_it)
            {
                geometry_msgs::Point32 point;
                point.x = p_it->x;
                point.y = p_it->y;
                point.z = p_it->z;
                plane_cloud_.points.push_back(point);
                plane_cloud_.channels.at(0).values.push_back(plane_intensity);
            }

            // Check if we found a box:
            if (checkClusterForBox(cloud_cluster))
            {
                const Box & box = createBox(cloud_cluster);
                publishCollisionObject(box);
                publishActionFeedback(box);

                ROS_INFO_STREAM("FOUND BOX! Name: " << box.name_ << "; Area: " << computeArea(cloud_cluster)  << "; Pose: " << box.pose_.pose);
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
    ROS_INFO("BoxDetection action finished; found %d boxes", boxes_.size());
}


void BoxDetectionNode::publisherTimerCallback(const ros::TimerEvent &)
{
    plane_cloud_publisher_.publish(plane_cloud_);
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
    // Compute eigen vectors:
    pcl::PCA<PclPoint> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    // Transform cloud into eigen vector space:
    PclPointCloud projected_cloud;
    pca.project(*cloud, projected_cloud);

    // Get size of projected plane:
    PclPoint min_pt;
    PclPoint max_pt;
    pcl::getMinMax3D(projected_cloud, min_pt, max_pt);
    PclPoint size = max_pt - min_pt;
    size.z = size.y; // We don't get the height from the plane, so we simply assume a square base

    // Compute center = center of plane minus height/2
    Eigen::Vector3f center = pca.getMean().topRows(3) + eigen_vectors.transpose() * Eigen::Vector3f(0.0f, 0.0f, -size.z / 2.0f);

    // Compute orientation (for now, we assume the box to lie flat on the ground, I didn't want to think about quaternion computations...):
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
    box.pose_.pose.position.x = pca.getMean()(0);
    box.pose_.pose.position.y = pca.getMean()(1);
    box.pose_.pose.position.z = pca.getMean()(2);
    box.pose_.pose.orientation.x = orientation.x();
    box.pose_.pose.orientation.y = orientation.y();
    box.pose_.pose.orientation.z = orientation.z();
    box.pose_.pose.orientation.w = orientation.w();

    box.size_.x = size.x;
    box.size_.y = size.y;
    box.size_.z = size.z;

    boxes_.push_back(box);

    return boxes_.back();
}

void BoxDetectionNode::publishCollisionObject(const Box & box)
{
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = box.size_.x;
    primitive.dimensions[1] = box.size_.y;
    primitive.dimensions[2] = box.size_.z;

    moveit_msgs::CollisionObject object;
    object.header = box.pose_.header;
    object.id = box.name_;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box.pose_.pose);
    object.operation = moveit_msgs::CollisionObject::ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(object);
    planning_scene_publisher_.publish(planning_scene);
}

void BoxDetectionNode::publishActionFeedback(const Box & box){
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
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        return 1;
    }
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
