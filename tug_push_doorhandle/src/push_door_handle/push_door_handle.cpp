#include<tug_push_door_handle/push_door_handle/push_door_handle.h>

namespace push_door_handle
{

PushDoorHandle::PushDoorHandle()
{
    ROS_ERROR("constru Push door handle");
    std::string planning_group_name = "right_arm";
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
    move_group_->setPlanningTime(180.0);


    ros::NodeHandle nh;

    display_publisher_ = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
//    desired_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/push_door_handle/ddesired_pose",1,true);
//    current_right_hand_pose_ = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state",1, &PushDoorHandle::stateCallback, this);

}

//void PushDoorHandle::stateCallback(const baxter_core_msgs::EndpointStateConstPtr& msg)
//{
//    current_pose_.pose = msg->pose;
//    current_pose_.header = msg->header;
//    current_pose_.header.frame_id = "base";
//}

bool PushDoorHandle::pushHandle()
{
    geometry_msgs::PoseStamped prev_pose = move_group_->getCurrentPose("right_wrist");

    desired_pose_ = move_group_->getCurrentPose("right_wrist");
    desired_pose_.pose.position.z = desired_pose_.pose.position.z - 0.08;
//    desired_pose_pub_.publish(desired_pose_);
    move_group_->setPoseTarget(desired_pose_);

    //planning
    moveit::planning_interface::MoveGroup::Plan push_plan;
    move_group_->plan(push_plan);

    //RViz visualization
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = push_plan.start_state_;
    display_trajectory.trajectory.push_back(push_plan.trajectory_);
    display_publisher_.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    bool push_handle = move_group_->execute(push_plan);

    //push handle was successfull move arm back
    if(push_handle)
    {
        move_group_->setPoseTarget(prev_pose);
        moveit::planning_interface::MoveGroup::Plan back_plan;
        move_group_->plan(back_plan);

        //RViz visualization
        moveit_msgs::DisplayTrajectory display_trajectory_back;
        display_trajectory_back.trajectory_start = back_plan.start_state_;
        display_trajectory_back.trajectory.push_back(back_plan.trajectory_);
        display_publisher_.publish(display_trajectory_back);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);
        push_handle = move_group_->execute(back_plan);
    }
    return push_handle;

}

PushDoorHandle::~PushDoorHandle(){}


}
