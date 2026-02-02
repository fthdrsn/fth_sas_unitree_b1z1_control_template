/*
#    Copyright (c) 2025-2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#   This example is based on https://github.com/MarinhoLab/sas_robot_driver_ur
#
# ################################################################
*/

#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp"

using namespace DQ_robotics;

#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_unitree_b1z1_control_template_joint_interface_example");

    // 1 ms clock
    sas::Clock clock{0.001};
    clock.init();

    // Initialize the RobotDriverClient
    sas::UnitreeB1Z1RobotClient rdi(node, "sas_b1/b1_1", "sas_z1/z1_1", sas::UnitreeB1Z1RobotClient::MODE::CONTROL);

    // Wait for RobotDriverClient to be enabled
    while(!rdi.is_enabled() && !kill_this_process)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node);
    }

    // Get topic information
    RCLCPP_INFO_STREAM(node->get_logger(),"b1_topic_prefix = " << rdi.get_b1_topic_prefix());
    RCLCPP_INFO_STREAM(node->get_logger(),"z1_topic_prefix = " << rdi.get_z1_topic_prefix());

    // Read the values sent by the RobotDriverServer


    // For some iterations. Note that this can be stopped with CTRL+C.
    VectorXd qarm = rdi.get_arm_joint_states_including_gripper();

    for(auto i=0;i<5000;++i)
    {
        clock.update_and_sleep();
        VectorXd u_base = (VectorXd(3) << 0.01, 0.01, 0.1).finished();
        rdi.set_target_b1_planar_joint_velocities(u_base);

        qarm(0) = -pi/2;
        rdi.set_arm_joint_positions(qarm);

        rclcpp::spin_some(node);
    }
    rdi.set_target_b1_planar_joint_velocities((VectorXd(3) << 0, 0, 0).finished());


    // Statistics
    RCLCPP_INFO_STREAM(node->get_logger(),"Statistics for the entire loop");

    RCLCPP_INFO_STREAM(node->get_logger(),"  Mean computation time: " << clock.get_statistics(
                                               sas::Statistics::Mean, sas::Clock::TimeType::Computational));
    RCLCPP_INFO_STREAM(node->get_logger(),"  Mean idle time: " << clock.get_statistics(
                                               sas::Statistics::Mean, sas::Clock::TimeType::Idle));
    RCLCPP_INFO_STREAM(node->get_logger(),"  Mean effective thread sampling time: " << clock.get_statistics(
                                               sas::Statistics::Mean, sas::Clock::TimeType::EffectiveSampling));

}
