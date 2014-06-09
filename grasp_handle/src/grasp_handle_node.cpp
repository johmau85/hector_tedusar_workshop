/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CU Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \brief   Simple pick place for blocks using Baxter
 * \author  Dave Coleman
 */

// ROS
#include <ros/ros.h>
#include <grasp_handle/grasp/simple_grasp.h>
#include <tug_grasp_object_msgs/GraspObjectAction.h>

typedef boost::shared_ptr<actionlib::SimpleActionClient<tug_grasp_object_msgs::GraspObjectAction> > GraspObjectActionClientPtr;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "grasp_handle");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Start the pick place node
    //  simple_grasp::Grasp grasp_handle;
    //  grasp_handle.startRoutine("door_handle");

    GraspObjectActionClientPtr grasp_client;

    grasp_client.reset(new actionlib::SimpleActionClient<tug_grasp_object_msgs::GraspObjectAction>("grasp", false));

    while(!grasp_client->waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for the grasp action server to come up");
      }

    tug_grasp_object_msgs::GraspObjectGoal goal;
    tug_grasp_object_msgs::GraspObject object;
    object.object_name = "door_handle";
    object.object_type = "door_handle";


    geometry_msgs::Pose handle_pose;
    handle_pose.orientation.w = -0.233035946881;
    handle_pose.orientation.x = 0.663621432006;
    handle_pose.orientation.y = 0.326102320682 ;
    handle_pose.orientation.z = 0.631631315633;
    handle_pose.position.x = 0.99;
    handle_pose.position.y = -0.36;
    handle_pose.position.z = 0.15;



    object.object_pose.pose = handle_pose;
    object.object_pose.header.frame_id = "/base";
    object.object_pose.header.stamp = ros::Time::now();

    goal.object_to_grasp = object;

    grasp_client->sendGoal(goal);

    while(true)
    {
        actionlib::SimpleClientGoalState grasp_state = grasp_client->getState();
//        ROS_ERROR_STREAM("running in while" );
        if(!((grasp_state == actionlib::SimpleClientGoalState::PENDING) || (grasp_state == actionlib::SimpleClientGoalState::ACTIVE)  || (grasp_state == actionlib::SimpleClientGoalState::SUCCEEDED)))
        {
            if((grasp_state == actionlib::SimpleClientGoalState::RECALLED) || (grasp_state == actionlib::SimpleClientGoalState::PREEMPTED))
            {
                ROS_ERROR_STREAM("The action was canceled during state: " );
            }
            else if(grasp_state == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_ERROR_STREAM("The action reported an execution error during state: " );
            }
            else if(grasp_state == actionlib::SimpleClientGoalState::REJECTED)
            {
                ROS_ERROR_STREAM("The action was rejected in state: " );
            }
            else
            {
                ROS_ERROR_STREAM("Unhandled action state." );
            }
            break;

        }

        if(grasp_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR_STREAM("Action sucessfull complited" );
            break;
        }
    }


    ros::shutdown();

    return 0;
}
