/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Johannes Maurer, Alexander Buchegger
 *                      Institute for Software Technology,
 *                      Graz University of Technology
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
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
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
 *
 *********************************************************************/

#include <schunk_lwa4p_control/schunk_lwa4p_control.hpp>
#include <ipa_canopen_core/canopen.h>
#include <controller_manager/controller_manager.h>
#include <stdexcept>

namespace schunk_lwa4p_control
{

SchunkLWA4P::SchunkLWA4P()
{
    ros::NodeHandle private_nh("~");

    std::string device;
    private_nh.param<std::string>("device", device, std::string("/dev/pcan32"));

    int sync_interval;
    private_nh.param<int>("sync_interval", sync_interval, 10);
    canopen::syncInterval = std::chrono::milliseconds(static_cast<int>(sync_interval));

    std::string baudrate;
    private_nh.param<std::string>("baudrate", baudrate, std::string("500K"));
    canopen::baudRate = baudrate;

    std::string chain_name;
    private_nh.param<std::string>("chain_name", chain_name, std::string("arm_controller"));


    std::vector<std::string> joint_names;

    std::string param = chain_name + "/joint_names";
    XmlRpc::XmlRpcValue joint_names_xmlrpc;
    if (private_nh.hasParam(param))
    {
        private_nh.getParam(param, joint_names_xmlrpc);
    }
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
        throw std::invalid_argument(param);
    }

    for (int i=0; i<joint_names_xmlrpc.size(); i++)
    {
        joint_names.push_back(static_cast<std::string>(joint_names_xmlrpc[i]));
    }

    int DOF = joint_names.size();

    param = chain_name + "/module_ids";
    XmlRpc::XmlRpcValue module_ids_xmlrpc;
    if (private_nh.hasParam(param))
    {
        private_nh.getParam(param, module_ids_xmlrpc);
    }
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
        throw std::invalid_argument(param);
    }

    if( module_ids_xmlrpc.size() != DOF)
    {
        ROS_ERROR("The size of the ids parameter is different from the size of the degrees of freedom. Shutting down node...");
        throw std::invalid_argument(param);
    }

    std::vector<uint8_t> module_ids;
    for (int i=0; i<module_ids_xmlrpc.size(); i++)
    {
        module_ids.push_back(static_cast<int>(module_ids_xmlrpc[i]));
    }

    for (unsigned int i=0; i<joint_names.size(); i++)
    {
        canopen::devices[ module_ids[i] ] = canopen::Device(module_ids[i], joint_names[i], chain_name);
    }

    canopen::deviceGroups[ chain_name ] = canopen::DeviceGroup(module_ids, joint_names);

    canopen::sendPos = canopen::defaultPDOOutgoing_interpolated;
    for(auto it : canopen::devices)
    {
        canopen::incomingPDOHandlers[ 0x180 + it.first ] = [it](const TPCANRdMsg mS) { canopen::defaultPDO_incoming_status( it.first, mS ); };
        canopen::incomingPDOHandlers[ 0x480 + it.first ] = [it](const TPCANRdMsg mP) { canopen::defaultPDO_incoming_pos( it.first, mP ); };
        canopen::incomingEMCYHandlers[ 0x081 + it.first ] = [it](const TPCANRdMsg mE) { canopen::defaultEMCY_incoming( it.first, mE ); };
    }

    bool init_success = canopen::init(device, chain_name, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if(!init_success)
    {
        ROS_ERROR("This chain could not be initialized. Check for possible errors and try to initialize it again.");
        throw std::runtime_error("This chain could not be initialized. Check for possible errors and try to initialize it again.");
    }


//    // connect and register the joint state interface
//    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
//    jnt_state_interface.registerHandle(state_handle_a);

//    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
//    jnt_state_interface.registerHandle(state_handle_b);

//    registerInterface(&jnt_state_interface);

//    // connect and register the joint position interface
//    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
//    jnt_pos_interface.registerHandle(pos_handle_a);

//    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
//    jnt_pos_interface.registerHandle(pos_handle_b);

//    registerInterface(&jnt_pos_interface);
}

}  // end namespace schunk_lwa4p_control

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "schunk_lwa4p_control");

        schunk_lwa4p_control::SchunkLWA4P schunk_lwa4p;

        controller_manager::ControllerManager cm(&schunk_lwa4p);

        double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(canopen::syncInterval).count();

        ros::Rate loop_rate(lr);

        ros::Time last_time = ros::Time::now();

        while (ros::ok())
        {
            loop_rate.sleep();
            ros::Time current_time = ros::Time::now();
            ros::Duration elapsed_time = ros::Duration(current_time - last_time);

//            schunk_lwa4p.read();
            cm.update(current_time, elapsed_time);
//            schunk_lwa4p.write();

            last_time = current_time;
            ros::spinOnce();
        }
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}

