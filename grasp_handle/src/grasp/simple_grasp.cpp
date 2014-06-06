#include <grasp_handle/grasp/simple_grasp.h>

namespace simple_grasp
{
Grasp::Grasp(): auto_reset_(false),
    auto_reset_sec_(4),
    arm_("right"),
    planning_group_name_(arm_+"_arm")
{
    ros::NodeHandle nh;

    // Create MoveGroup for one of the planning groups
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(30.0);
//    move_group_->setGoalTolerance(0.02);

    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh, arm_+"_hand"))
        ros::shutdown();

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools( grasp_data_.base_link_));
    visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);

    simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

    // Let everything load
    ros::Duration(1.0).sleep();

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
        return;

    // Show grasp visualizations or not
    visual_tools_->setMuted(true);

    // Create the walls and tables
    createEnvironment(visual_tools_);
}

Grasp::~Grasp()
{}

bool Grasp::startRoutine(std::string name)
{

    // --------------------------------------------------------------------------------------------------------
    // Repeat pick and place forever

        // -------------------------------------------------------------------------------------
        // Send Baxter to neutral position
        //if( !baxter_util_.positionBaxterNeutral() )
        //  return false;

        // --------------------------------------------------------------------------------------------



                // Pick -------------------------------------------------------------------------------------
                while(ros::ok())
                {


                    if( !pick(name) )
                    {
                        ROS_ERROR_STREAM_NAMED("pick_place","Pick failed.");

                        // Ask user if we should try again
                        if( !promptUser("Pick failed. Retry? ") )
                            exit(0);

                    }
                    else
                    {
                        ROS_INFO_STREAM_NAMED("pick_place","Done with pick ---------------------------");
                        break;
                    }
                }



        // Ask user if we should repeat
        //if( !promptUser() )
        //  break;
        ros::Duration(1.0).sleep();
}



    //    // Move to gravity neutral position
    //    //if( !baxter_util_.positionBaxterNeutral() )
    //    //  return false;

    //    // Everything worked!
    //    return true;
    //  }

    void Grasp::resetBlock(MetaObject block)
    {
        // Remove attached object
        //    visual_tools_->cleanupACO(block.name);

        //    // Remove collision object
        //    visual_tools_->cleanupCO(block.name);

        //    // Add the collision block
        //    visual_tools_->publishCollisionBlock(block.start_pose, block.name, BLOCK_SIZE);
    }

    MetaObject Grasp::createStartBlock(double x, double y, const std::string name)
    {
        MetaObject start_block;
        //    start_block.name = name;

        //    // Position
        //    start_block.start_pose.position.x = x;
        //    start_block.start_pose.position.y = y;
        //    start_block.start_pose.position.z = getTableHeight(-0.9);

        //    // Orientation
        //    double angle = 0; // M_PI / 1.5;
        //    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
        //    start_block.start_pose.orientation.x = quat.x();
        //    start_block.start_pose.orientation.y = quat.y();
        //    start_block.start_pose.orientation.z = quat.z();
        //    start_block.start_pose.orientation.w = quat.w();

        return start_block;
    }

    bool Grasp::pick(std::string name)
    {
        std::vector<moveit_msgs::Grasp> grasps;

        //    // Pick grasp

        geometry_msgs::Pose handle_pose;
        handle_pose.orientation.w = -0.233035946881;
        handle_pose.orientation.x = 0.663621432006;
        handle_pose.orientation.y = 0.326102320682 ;
        handle_pose.orientation.z = 0.631631315633;
        handle_pose.position.x = 1.01;
        handle_pose.position.y = -0.34;
        handle_pose.position.z = 0.15;






        simple_grasps_->generateBlockGrasps( handle_pose, grasp_data_, grasps );
        ROS_ERROR_STREAM("Genreated " << grasps.size() << " grap(s)");

        //    // Prevent collision with table
        move_group_->setSupportSurfaceName("wall_left");
        move_group_->setSupportSurfaceName("wall_right");
        move_group_->setSupportSurfaceName("wall_top");
        move_group_->setSupportSurfaceName("door");


        //    // Allow blocks to be touched by end effector
        //    {
        //      // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
        std::vector<std::string> allowed_touch_objects;
        allowed_touch_objects.push_back(SUPPORT_SURFACE1_NAME);


        //      // Add this list to all grasps
        for (std::size_t i = 0; i < grasps.size(); ++i)
        {
            grasps[i].allowed_touch_objects = allowed_touch_objects;
            moveit_msgs::Grasp grasp;
            grasp.post_grasp_retreat.min_distance = 0.0;
            grasp.post_grasp_retreat.desired_distance = 0.0;
            grasp.post_grasp_retreat.direction.vector.x = 0.0;
            grasp.post_grasp_retreat.direction.vector.y = 0.0;
            grasp.post_grasp_retreat.direction.vector.z = 0.0;
            grasps[i].post_grasp_retreat = grasp.post_grasp_retreat;
        }
        //    }

        //    //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << grasps[0]);
        moveit_msgs::Grasp my_grasp;
//        ROS_ERROR_STREAM_NAMED("simle_grasp -> grasp Pose", grasp.grasp_pose);
//        ROS_ERROR_STREAM_NAMED("simle_grasp -> pre grasp Approach", grasp.pre_grasp_approach);
//        ROS_ERROR_STREAM_NAMED("simle_grasp -> pre grasp Posture", grasp.pre_grasp_posture);
//        ROS_ERROR_STREAM_NAMED("simle_grasp -> post grasp Retreat", grasp.post_grasp_retreat);


        geometry_msgs::PoseStamped grasp_pose;
        grasp_pose.pose.orientation.w = -0.395028916185;
        grasp_pose.pose.orientation.x = 0.394144890472;
        grasp_pose.pose.orientation.y = -0.533862946359;
        grasp_pose.pose.orientation.z = 0.635289158728;
        grasp_pose.pose.position.x = 0.91702215543;
        grasp_pose.pose.position.y = -0.361242632643;
        grasp_pose.pose.position.z =0.129415041813;
        grasp_pose.header.frame_id = grasp_data_.base_link_;
        grasp_pose.header.stamp =ros::Time::now();
        my_grasp.grasp_pose = grasp_pose;

        my_grasp.post_grasp_retreat.min_distance = 0.0;
        my_grasp.post_grasp_retreat.desired_distance = 0.0;
        my_grasp.post_grasp_retreat.direction.vector.x = 0.0;
        my_grasp.post_grasp_retreat.direction.vector.y = 0.0;
        my_grasp.post_grasp_retreat.direction.vector.z = 0.0;

        my_grasp.pre_grasp_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
        my_grasp.pre_grasp_approach.min_distance = grasp_data_.approach_retreat_min_dist_;







        return move_group_->pick(name, grasps);
    }

    bool Grasp::place(const geometry_msgs::Pose& goal_block_pose, std::string block_name)
    {
        //    ROS_WARN_STREAM_NAMED("place","Placing '"<< block_name << "'");

        //    std::vector<moveit_msgs::PlaceLocation> place_locations;
        //    std::vector<moveit_msgs::Grasp> grasps;

        //    // Re-usable datastruct
        //    geometry_msgs::PoseStamped pose_stamped;
        //    pose_stamped.header.frame_id = grasp_data_.base_link_;
        //    pose_stamped.header.stamp = ros::Time::now();

        //    // Create 360 degrees of place location rotated around a center
        //    for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
        //    {
        //      pose_stamped.pose = goal_block_pose;

        //      // Orientation
        //      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
        //      pose_stamped.pose.orientation.x = quat.x();
        //      pose_stamped.pose.orientation.y = quat.y();
        //      pose_stamped.pose.orientation.z = quat.z();
        //      pose_stamped.pose.orientation.w = quat.w();

        //      // Create new place location
        //      moveit_msgs::PlaceLocation place_loc;

        //      place_loc.place_pose = pose_stamped;

        //      visual_tools_->publishBlock( place_loc.place_pose.pose, moveit_visual_tools::BLUE, BLOCK_SIZE);

        //      // Approach
        //      moveit_msgs::GripperTranslation pre_place_approach;
        //      pre_place_approach.direction.header.stamp = ros::Time::now();
        //      pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
        //      pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
        //      pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
        //      pre_place_approach.direction.vector.x = 0;
        //      pre_place_approach.direction.vector.y = 0;
        //      pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
        //      place_loc.pre_place_approach = pre_place_approach;

        //      // Retreat
        //      moveit_msgs::GripperTranslation post_place_retreat;
        //      post_place_retreat.direction.header.stamp = ros::Time::now();
        //      post_place_retreat.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
        //      post_place_retreat.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
        //      post_place_retreat.direction.header.frame_id = grasp_data_.base_link_;
        //      post_place_retreat.direction.vector.x = 0;
        //      post_place_retreat.direction.vector.y = 0;
        //      post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
        //      place_loc.post_place_retreat = post_place_retreat;

        //      // Post place posture - use same as pre-grasp posture (the OPEN command)
        //      place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

        //      place_locations.push_back(place_loc);
        //    }

        //    // Prevent collision with table
        //    move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

        //    move_group_->setPlannerId("RRTConnectkConfigDefault");

        //    return move_group_->place(block_name, place_locations);
    }

    bool Grasp::promptUser(std::string string)
    {
        // Make sure ROS is still with us
        if( !ros::ok() )
            return false;

        if( auto_reset_ )
        {
            ROS_INFO_STREAM_NAMED("pick_place","Auto-retrying in " << auto_reset_sec_ << " seconds");
            ros::Duration(auto_reset_sec_).sleep();
        }
        else
        {
            ROS_INFO_STREAM_NAMED("pick_place",string.c_str() << " (y/n)");
            char input; // used for prompting yes/no
            std::cin >> input;
            if( input == 'n' )
                return false;
        }
        return true;
    }

} //namespace
