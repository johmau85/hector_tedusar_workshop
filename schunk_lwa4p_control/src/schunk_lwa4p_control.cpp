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

    private_nh.param<std::string>("chain_name", chain_name_, std::string("arm_controller"));

    std::vector<std::string> joint_names;

    std::string param = chain_name_ + "/joint_names";
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

    param = chain_name_ + "/module_ids";
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
        canopen::devices[ module_ids[i] ] = canopen::Device(module_ids[i], joint_names[i], chain_name_);
    }

    canopen::deviceGroups[ chain_name_ ] = canopen::DeviceGroup(module_ids, joint_names);

    canopen::sendPos = canopen::defaultPDOOutgoing_interpolated;
    for(auto it : canopen::devices)
    {
        canopen::incomingPDOHandlers[ 0x180 + it.first ] = [it](const TPCANRdMsg mS) { canopen::defaultPDO_incoming_status( it.first, mS ); };
        canopen::incomingPDOHandlers[ 0x480 + it.first ] = [it](const TPCANRdMsg mP) { canopen::defaultPDO_incoming_pos( it.first, mP ); };
        canopen::incomingEMCYHandlers[ 0x081 + it.first ] = [it](const TPCANRdMsg mE) { canopen::defaultEMCY_incoming( it.first, mE ); };
    }

    bool init_success = canopen::init(device, chain_name_, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if(!init_success)
    {
        ROS_ERROR("This chain could not be initialized. Check for possible errors and try to initialize it again.");
        throw std::runtime_error("This chain could not be initialized. Check for possible errors and try to initialize it again.");
    }

    for(unsigned int i=0; i<joint_names.size(); i++)
    {
        joint_vel_cmds_[joint_names[i]] = 0.0;
        joint_positions_[joint_names[i]] = 0.0;
        joint_velocitys_[joint_names[i]] = 0.0;
        joint_efforts_[joint_names[i]] = 0.0;

        hardware_interface::JointStateHandle state_handle(joint_names[i], &joint_positions_[joint_names[i]], &joint_velocitys_[joint_names[i]], &joint_efforts_[joint_names[i]]);
        joint_state_interface_.registerHandle(state_handle);


        hardware_interface::JointHandle vel_handle(joint_state_interface_.getHandle(joint_names[i]), &joint_vel_cmds_[joint_names[i]]);
        velocity_joint_interface_.registerHandle(vel_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
}

void SchunkLWA4P::read()
{
    canopen::DeviceGroup dg = canopen::deviceGroups[chain_name_];

    for (auto id : dg.getCANids())
    {
        std::string joint_name = canopen::devices[id].getName();
        joint_positions_[joint_name] = canopen::devices[id].getActualPos();
        joint_velocitys_[joint_name] = canopen::devices[id].getActualVel();
    }
}

void SchunkLWA4P::write()
{
    std::vector<double> velocities;

    for( std::map<std::string, double>::iterator it = joint_vel_cmds_.begin(); it != joint_vel_cmds_.end(); ++it ) {
        velocities.push_back( it->second );
    }

    canopen::deviceGroups[chain_name_].setVel(velocities);
}

}  // end namespace schunk_lwa4p_control

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "schunk_lwa4p_control");

        schunk_lwa4p_control::SchunkLWA4P schunk_lwa4p;

        controller_manager::ControllerManager cm(&schunk_lwa4p);

        ros::AsyncSpinner spinner(4);
        spinner.start();

        double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(canopen::syncInterval).count();

        ros::Rate loop_rate(lr);

        ros::Time last_time = ros::Time::now();

        while (ros::ok())
        {
            loop_rate.sleep();

            ros::Time current_time = ros::Time::now();
            ros::Duration elapsed_time = current_time - last_time;
            last_time = current_time;

            schunk_lwa4p.read();
            cm.update(current_time, elapsed_time);
            schunk_lwa4p.write();
        }
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}

