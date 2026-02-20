/*
#    Copyright (c) 2025 Adorno-Lab
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
#
# ################################################################
*/


#pragma once
#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/DQ.h>
#include <sas_core/sas_clock.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>

using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{
struct ControlExampleConfiguration
{
    std::string cs_host;
    int cs_port;
    int cs_TIMEOUT_IN_MILISECONDS;
    std::string cs_B1_robotname;
    std::string cs_Z1_robotname;
    std::string B1_topic_prefix;
    std::string Z1_topic_prefix;
    double thread_sampling_time_sec;
    std::string vfi_file;

    std::string external_B1_agent_topic_prefix;
    std::string external_Z1_agent_topic_prefix;
    std::string external_agent_3_topic_prefix;
    std::string task_commander_topic_prefix;
    int agent_index;
    int number_of_agents;
    //DQ desired_relative_pose;
};

class ControlExample
{

private:

    //double timer_period_;
    double T_;
    sas::Clock clock_;


    //---------- To get data about the robot state and command the robot----------------//

    //std::shared_ptr<UnitreeB1Z1RobotClient> robot_client_;

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_frame_x_;
    //-----------------------------------------------------------------------------


protected:


    ControlExampleConfiguration configuration_;
    std::atomic_bool* st_break_loops_;
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<std::string> z1_jointnames_;
    bool robot_reached_region_;

    bool _should_shutdown() const;
    void _connect();
    void _update_kinematic_model();
    void _publish_coppeliasim_frame_x(const DQ& pose);
    void _publish_control_inputs(const VectorXd& u);

public:
    ControlExample(const ControlExample&)=delete;
    ControlExample()=delete;
    ~ControlExample();

    ControlExample(std::shared_ptr<Node>& node,
                   const ControlExampleConfiguration &configuration,
                   std::atomic_bool* break_loops);


    void control_loop();
};



}
