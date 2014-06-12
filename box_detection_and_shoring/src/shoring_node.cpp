#include <box_detection_and_shoring/shoring_node.h>
#include <iostream>
#include <stdexcept>
#include <boost/make_shared.hpp>
#include <controller_manager_msgs/SwitchController.h>

namespace box_detection_and_shoring
{

ShoringNode::ShoringNode()
    : place_block_action_client_("place_block", true), box_detection_action_client_("detect_boxes", true)
{
    ros::NodeHandle private_nh("~");

    std::string group;
    private_nh.param<std::string>("group", group, "lwa4p_arm");
    private_nh.param<std::string>("heap_frame_id", heap_frame_id_, "heap"); // TODO: static transform publisher for this frame

    scan_pose_.header.frame_id = heap_frame_id_;
    scan_pose_.header.stamp = ros::Time(0);
    scan_pose_.pose.position.x = 0.0;
    scan_pose_.pose.position.y = 0.0;
    scan_pose_.pose.position.z = 0.6;
    scan_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI, 0.0);

    ROS_INFO("Waiting for block placing action server...");
    place_block_action_client_.waitForServer();

    ROS_INFO("Waiting for box detection action server...");
    box_detection_action_client_.waitForServer();

    move_group_ = boost::make_shared<moveit::planning_interface::MoveGroup>(group);
    planning_frame_id_ = move_group_->getPlanningFrame();

    switch_controller_service_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    grasping_done_subscriber_ = nh_.subscribe<std_msgs::Empty>("grasping_done", 1, &ShoringNode::graspingDoneCallback, this);
}

void ShoringNode::run()
{
    ROS_INFO("Starting shoring");

    while (ros::ok()) {
        // Move arm to scan pose:
        ROS_INFO("Moving arm to scanning pose");
        if (!moveToPose(scan_pose_) || !ros::ok())
            return;

        // Call box detection:
        ROS_INFO("Looking for boxes");
        geometry_msgs::PoseStamped box_pose;
        if (!detectBox(box_pose) || !ros::ok())
            return;

        // Compute pre-grasp pose:
        geometry_msgs::PoseStamped pre_grasp_pose;
        tf_listener_.transformPose(heap_frame_id_, box_pose, pre_grasp_pose);
        pre_grasp_pose.pose.position.z += 0.2;
        pre_grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI, 0.0);

        // Move arm to pre-grasp pose:
        ROS_INFO("Moving arm to pre-grasp pose");
        if (!moveToPose(pre_grasp_pose) || !ros::ok())
            return;

        // Switch arm controllers (enable cartesian):
        ROS_INFO("Switching to cartesian arm controller");
        if (!switchControllers(true) || !ros::ok())
            return;

        // Wait for OK from operator:
        ROS_INFO("Waiting for OK from operator");
        if (!waitUntilGraspingDone() || !ros::ok())
            return;

        // Switch arm controller (disable cartesian):
        ROS_INFO("Switching back from cartesian arm controller");
        if (!switchControllers(false) || !ros::ok())
            return;

        // Compute post-grasp pose:
        geometry_msgs::PoseStamped end_effector_pose;
        end_effector_pose.header.frame_id = move_group_->getEndEffectorLink();
        end_effector_pose.header.stamp = ros::Time(0);
        end_effector_pose.pose.position.x = 0;
        end_effector_pose.pose.position.y = 0;
        end_effector_pose.pose.position.z = 0;
        end_effector_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        geometry_msgs::PoseStamped post_grasp_pose;
        tf_listener_.transformPose(heap_frame_id_, end_effector_pose, post_grasp_pose);
        post_grasp_pose.pose.position.z += 0.2;

        // Move arm to post-grasp position:
        ROS_INFO("Moving arm to post-grasp position");
        if (!moveToPose(post_grasp_pose) || !ros::ok())
            return;

        // Call shoring behaviour (place_block action):
        ROS_INFO("Placing block");
        if (!placeBlock() || !ros::ok())
            return;
    }
}

bool ShoringNode::detectBox(geometry_msgs::PoseStamped & box_pose)
{
    box_detection::BoxDetectionGoal goal;
    box_detection_action_client_.sendGoal(goal);

    while(!box_detection_action_client_.waitForResult(ros::Duration(5.0)))
    {
        if (!ros::ok())
            return false;
    }

    if (box_detection_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Box detection failed");
        return false;
    }

    box_detection::BoxDetectionResultConstPtr result = box_detection_action_client_.getResult();
    if (result->box_poses.empty())
    {
        ROS_ERROR("No boxes detected");
        return false;
    }

    box_pose = result->box_poses.at(0);
    return true;
}

bool ShoringNode::switchControllers(bool switch_to_cartesian)
{
    controller_manager_msgs::SwitchController srv;

    if (switch_to_cartesian)
    {
        srv.request.start_controllers.push_back("cartesian_controller");
        srv.request.stop_controllers.push_back("arm_controller");
    }
    else
    {
        srv.request.start_controllers.push_back("arm_controller");
        srv.request.stop_controllers.push_back("cartesian_controller");
    }

    if (!switch_controller_service_client_.call(srv))
    {
        ROS_ERROR("Unable to switch controllers");
        return false;
    }

    return true;
}

bool ShoringNode::placeBlock()
{
    shoring_msgs::PlaceBlockGoal goal;
    place_block_action_client_.sendGoal(goal);

    while(!place_block_action_client_.waitForResult(ros::Duration(5.0)))
    {
        if (!ros::ok())
            return false;
    }

    if (place_block_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR("Box placing failed");
        return false;
    }

    return true;
}

bool ShoringNode::waitUntilGraspingDone()
{
    ros::spinOnce(); // Consume old messages

    grasping_done_ = false;
    while (!grasping_done_ && ros::ok())
        ros::spinOnce();

    return grasping_done_;
}

void ShoringNode::graspingDoneCallback(const std_msgs::EmptyConstPtr &)
{
    grasping_done_ = true;
}




bool ShoringNode::moveToPose(const geometry_msgs::PoseStamped & pose)
{
    geometry_msgs::PoseStamped transformed_pose;
    tf_listener_.transformPose(planning_frame_id_, pose, transformed_pose);

    move_group_->setPoseTarget(transformed_pose);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    while (!move_group_->plan(my_plan))
    {
        if (!ros::ok())
            return false;
    }

    /* Uncomment below line when working with a real robot*/
    bool result = move_group_->execute(my_plan);
    move_group_->clearPoseTarget();

    sleep(5.0);
    return result;
}

//void ShoringNode::moveArmCartesian(const geometry_msgs::PoseStamped & point_1, const geometry_msgs::PoseStamped & point_2s)
//{
//    std::vector<geometry_msgs::Pose> waypoints;
//    waypoints.push_back(point_1);
//    waypoints.push_back(point_2);

//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction = move_group_->computeCartesianPath(waypoints,
//                                                        0.02,  // eef_step
//                                                        0.0,   // jump_threshold
//                                                        trajectory);

//    robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(), move_group_->getName());
//    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
//    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.01);

//    bool success = iptp.computeTimeStamps(rt);
//    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

//    // Get RobotTrajectory_msg from RobotTrajectory
//    moveit_msgs::RobotTrajectory smooth_trajectory;
//    rt.getRobotTrajectoryMsg(smooth_trajectory);

//    // Finally plan and execute the trajectory
//    moveit::planning_interface::MoveGroup::Plan plan;
//    plan.trajectory_ = smooth_trajectory;

//    move_group_->execute(plan);
//    sleep(5.0);
//}

} // namespace




int main(int argc, char ** argv)
{
    ros::init(argc, argv, "shoring");

    try
    {
        box_detection_and_shoring::ShoringNode node;
        node.run();
        ros::spin();
        return 0;
    }
    catch (std::exception & ex)
    {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unhandled exception" << std::endl;
        return 1;
    }
}
