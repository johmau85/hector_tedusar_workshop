#include<tug_grasp_object/grasp_object_node.h>
#include<tug_grasp_object/grasp_object/grasp_door_handle.h>
namespace grasp_object_node
{

GraspObjectNode::GraspObjectNode() :
    nh_(),
    tf_listener_(),
    grasp_object_action_server_(nh_, "grasp", boost::bind( &GraspObjectNode::executeGraspObjectCB,this,_1),false)
{
    ros::NodeHandle private_nh("~");

    double wait_for_server_sec;
    private_nh.param<double>("wait_for_server_duration", wait_for_server_sec, 0.5);
    wait_for_server_duration_ = ros::Duration(wait_for_server_sec);

    private_nh.param<double>("control_rate", control_rate_, 20.0);

    grasp_object_action_server_.start();
}

void GraspObjectNode::executeGraspObjectCB(const tug_grasp_object_msgs::GraspObjectGoalConstPtr &goal)
{
    ROS_INFO("Grasping Object Started");
    tug_grasp_object_msgs::GraspObjectResult grasp_object_result;

    ros::Rate control_rate(control_rate_);

    while(true)
    {
        ROS_ERROR_STREAM("TEST true");
        if (grasp_object_action_server_.isPreemptRequested() || !ros::ok())
        {
            ROS_ERROR("Grasp object execution preempted.");

            grasp_object_result.result = tug_grasp_object_msgs::GraspObjectResult::EXECUTION_PREEMPTED;
            grasp_object_action_server_.setPreempted(grasp_object_result, std::string("Grasping object preempted"));

            return;
        }

        if (goal->object_to_grasp.object_type == "door_handle")
        {

            ROS_ERROR_STREAM("object type = "<< goal->object_to_grasp.object_type.c_str());
            grasp_object::GraspDoorHandle grasp_door_handle(goal->object_to_grasp.object_name, goal->object_to_grasp.object_pose, "right");
            bool result = grasp_door_handle.graspObject();
            ROS_ERROR_STREAM("Result is " << result);
            if(result)
            {
                grasp_object_result.result = tug_grasp_object_msgs::GraspObjectResult::GRASP_SUCCESS;
                grasp_object_action_server_.setSucceeded(grasp_object_result,"Grasping doorhandle successful");
                return;
            }
            else
            {
                grasp_object_result.result = tug_grasp_object_msgs::GraspObjectResult::GRASP_FAILED;
                grasp_object_action_server_.setAborted(grasp_object_result,"was not able to grasp doorhandle");
                return;
            }
        }
        else
        {
            grasp_object_result.result = tug_grasp_object_msgs::GraspObjectResult::EXECUTION_PREEMPTED;
            grasp_object_action_server_.setPreempted(grasp_object_result, std::string("Grasping object preempted due to not known object type"));
            return;
        }


    }

}

GraspObjectNode::~GraspObjectNode()
{}


}

int main(int argc, char** argv)
{
        try
        {
            ros::init(argc,argv,"grasp_object_node");

            grasp_object_node::GraspObjectNode node;

            ros::spin();
        }
        catch(...)
        {
            ROS_ERROR("Unhandled exception!");
            return -1;
        }

    return 0;
}
