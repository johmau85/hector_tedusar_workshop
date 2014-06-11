#include <ros/ros.h>

#include <tug_grasp_object_msgs/OpenDoorAction.h>

 #include <actionlib/client/simple_action_client.h>

typedef boost::shared_ptr<actionlib::SimpleActionClient<tug_grasp_object_msgs::OpenDoorAction> > OpenDoorActionClientPtr;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "push_door_handle");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Start the pick place node
    //  simple_grasp::Grasp grasp_handle;
    //  grasp_handle.startRoutine("door_handle");

    actionlib::SimpleActionClient<tug_grasp_object_msgs::OpenDoorAction> open_door_client("open_door", false);

//    push_door_handle_client.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction>("push_door_handle", false));

    while(!open_door_client.waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for the open door action server to come up");
      }

    tug_grasp_object_msgs::OpenDoorGoal goal;

    ROS_ERROR("send goal");

    goal.radius = 0.7;
    goal.opening_direction = "push";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.65892137268;
    pose.pose.position.y = -1.06749630216;
    pose.pose.position.z = 0.312122309527;
    pose.pose.orientation.w = 1.0;
    pose.header.frame_id = "base";
    pose.header.stamp = ros::Time::now();
    goal.midpoint_opening_circle = pose;
    open_door_client.sendGoal(goal);

    while(true)
    {
        actionlib::SimpleClientGoalState opening_door_state = open_door_client.getState();
//        ROS_ERROR_STREAM("running in while" );

        if(!((opening_door_state == actionlib::SimpleClientGoalState::PENDING) || (opening_door_state == actionlib::SimpleClientGoalState::ACTIVE)  || (opening_door_state == actionlib::SimpleClientGoalState::SUCCEEDED)))
        {
            if((opening_door_state == actionlib::SimpleClientGoalState::RECALLED) || (opening_door_state == actionlib::SimpleClientGoalState::PREEMPTED))
            {
                ROS_ERROR_STREAM("The action was canceled during state: " );
            }
            else if(opening_door_state == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_ERROR_STREAM("The action reported an execution error during state: " );
            }
            else if(opening_door_state == actionlib::SimpleClientGoalState::REJECTED)
            {
                ROS_ERROR_STREAM("The action was rejected in state: " );
            }
            else
            {
                ROS_ERROR_STREAM("Unhandled action state." );
            }
            break;

        }

        if(opening_door_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR_STREAM("Action sucessfull complited" );
            break;
        }
    }


    ros::shutdown();

    return 0;
}
