#include <tug_open_door/open_door/open_door.h>

namespace open_door
{
OpenDoor::OpenDoor()
{
    std::string planning_group_name = "right_arm";
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
    move_group_->setPlanningTime(180.0);


    ros::NodeHandle nh;

    display_publisher_ = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    display_way_points_ = nh.advertise<geometry_msgs::PoseArray>("/open_door/waypoints",1,true);
    display_mid_point_ = nh.advertise<geometry_msgs::PoseStamped>("/open_door/midpoint",1,true);
}

OpenDoor::~OpenDoor(){}

bool OpenDoor::openDoor(geometry_msgs::PoseStamped midpoint, float radius, std::string direction)
{
    geometry_msgs::PoseStamped cur_pose = move_group_->getCurrentPose("right_wrist");

    tf::TransformListener listener;
    geometry_msgs::PoseStamped midpoint_base;
    try
    {
        listener.waitForTransform("base", midpoint.header.frame_id, midpoint.header.stamp, ros::Duration(10.0) );
        listener.transformPose("base", midpoint, midpoint_base);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    midpoint_base.pose.position.z = cur_pose.pose.position.z;

    float calc_radius = std::sqrt(std::pow(midpoint_base.pose.position.x - cur_pose.pose.position.x,2 ) + std::pow(midpoint_base.pose.position.y - cur_pose.pose.position.y,2 ));

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose point = cur_pose.pose;

    geometry_msgs::PoseArray array;
    array.header.frame_id = "base";
    array.header.stamp = ros::Time::now();

    ROS_ERROR_STREAM("radius " << calc_radius);
    double start_value = std::atan2(cur_pose.pose.position.y - midpoint_base.pose.position.y  , cur_pose.pose.position.x - midpoint_base.pose.position.x);

    double end_value = start_value - 1.0;
    for(double alpha = start_value; alpha >= end_value; alpha -= 0.01)
    {
        point.position.x = midpoint_base.pose.position.x + calc_radius * std::cos(alpha);
        point.position.y = midpoint_base.pose.position.y + calc_radius * std::sin(alpha);
        //Convert to TF



        //        tf::Quaternion tf_q = tf::createQuaternionFromYaw(alpha);

        //        // Convert back to Eigen
        //        geometry_msgs::Quaternion quat_msg;
        //        tf::quaternionTFToMsg(tf_q,quat_msg);
        //        point.orientation = quat_msg;

        waypoints.push_back(point);
        array.poses.push_back(point);
    }
    display_mid_point_.publish(midpoint_base);
    display_way_points_.publish(array);


    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(15.0);




    robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), "right_arm");
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.01);



    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    // Finally plan and execute the trajectory
    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = trajectory;
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
    sleep(5.0);
    move_group_->execute(plan);



    return true;




}


}
