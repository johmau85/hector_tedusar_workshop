#include <ros/ros.h>

#include <tug_grasp_object_msgs/PushDoorHandleAction.h>

 #include <actionlib/client/simple_action_client.h>

typedef boost::shared_ptr<actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction> > PushDoorHandleActionClientPtr;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "push_door_handle");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Start the pick place node
    //  simple_grasp::Grasp grasp_handle;
    //  grasp_handle.startRoutine("door_handle");

    actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction> push_door_handle_client("push_door_handle", false);

//    push_door_handle_client.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::PushDoorHandleAction>("push_door_handle", false));

    while(!push_door_handle_client.waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for the push handle action server to come up");
      }

    tug_grasp_object_msgs::PushDoorHandleGoal goal;

    ROS_ERROR("send goal");
    push_door_handle_client.sendGoal(goal);

    while(true)
    {
        actionlib::SimpleClientGoalState push_door_state = push_door_handle_client.getState();
//        ROS_ERROR_STREAM("running in while" );

        if(!((push_door_state == actionlib::SimpleClientGoalState::PENDING) || (push_door_state == actionlib::SimpleClientGoalState::ACTIVE)  || (push_door_state == actionlib::SimpleClientGoalState::SUCCEEDED)))
        {
            if((push_door_state == actionlib::SimpleClientGoalState::RECALLED) || (push_door_state == actionlib::SimpleClientGoalState::PREEMPTED))
            {
                ROS_ERROR_STREAM("The action was canceled during state: " );
            }
            else if(push_door_state == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_ERROR_STREAM("The action reported an execution error during state: " );
            }
            else if(push_door_state == actionlib::SimpleClientGoalState::REJECTED)
            {
                ROS_ERROR_STREAM("The action was rejected in state: " );
            }
            else
            {
                ROS_ERROR_STREAM("Unhandled action state." );
            }
            break;

        }

        if(push_door_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR_STREAM("Action sucessfull complited" );
            break;
        }
    }


    ros::shutdown();

    return 0;
}
