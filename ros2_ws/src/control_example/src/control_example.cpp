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

#include <sas_core/sas_clock.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>

#include "control_example/control_example.hpp"

namespace sas
{

class ControlExample::Impl
{
public:

    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::shared_ptr<DQ_robotics_extensions::RobotConstraintManager> robot_constraint_manager_;
    std::shared_ptr<DQ_robotics_extensions::VFIConfigurationFileYaml> vfi_config_yaml;
    std::shared_ptr<DQ_Kinematics> robot_model_;
    std::shared_ptr<UnitreeB1Z1MobileRobot> kin_mobile_manipulator_;
    std::shared_ptr<UnitreeB1Z1CoppeliaSimZMQRobot> robot_cs_;
    std::shared_ptr<UnitreeB1Z1RobotClient> robot_client_;
    std::shared_ptr<rclcpp::Node> node__;

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_frame_x_;
    //Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_b1_pose_;


    DQ xd_;
    bool new_coppeliasim_xd_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_xd_;



    std::vector<std::string> z1_jointnames_;
    sas::Clock clock_;
    Impl(std::shared_ptr<Node>& node, const double& thread_sampling_time_sec,
         std::string& B1_topic_prefix):
        node__{node}, clock_{thread_sampling_time_sec}
    {
        publisher_coppeliasim_frame_x_ = node__->create_publisher<geometry_msgs::msg::PoseStamped>(
            B1_topic_prefix + "/set/coppeliasim_frame_x", 1);

        subscriber_xd_ = node__->create_subscription<geometry_msgs::msg::PoseStamped>(
            B1_topic_prefix + "/get/coppeliasim_frame_xd", 1, std::bind(&Impl::callback_xd_state,
                      this, std::placeholders::_1)
            );
        //if (simulate_discrete_steps)
        // {
        //  publisher_coppeliasim_b1_pose_ = node__->create_publisher<geometry_msgs::msg::PoseStamped>(
        //      B1_topic_prefix + "/set/coppeliasim_b1_pose", 1);
        // }
    }

    void spin_and_update(const std::string& msg)
    {
        rclcpp::spin_some(node__);
        clock_.update_and_sleep();
        rclcpp::spin_some(node__);
        RCLCPP_INFO_STREAM(node__->get_logger(), msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void connect(const std::string& host, const int& port, const int&TIMEOUT_IN_MILISECONDS,
                 const std::string& B1_robotname, const std::string& Z1_robotname)
    {
        cs_->connect(host, port, TIMEOUT_IN_MILISECONDS);
        z1_jointnames_  = cs_->get_jointnames_from_object(Z1_robotname);
        robot_cs_ = std::make_shared<UnitreeB1Z1CoppeliaSimZMQRobot>(B1_robotname, cs_);
        rclcpp::spin_some(node__);
    }


    void publish_coppeliasim_frame_x(const DQ& pose)
    {
        geometry_msgs::msg::PoseStamped msg = sas::dq_to_geometry_msgs_pose_stamped(pose);
        publisher_coppeliasim_frame_x_->publish(msg);
    }

    void callback_xd_state(const geometry_msgs::msg::PoseStamped& msg)
    {
        xd_ = sas::geometry_msgs_pose_stamped_to_dq(msg);
        if (!is_unit(xd_))
        {
            RCLCPP_INFO_STREAM(node__->get_logger(), "::Problem reading desired pose");
            new_coppeliasim_xd_data_available_ = false;
        }
        new_coppeliasim_xd_data_available_ = true;
    }

    DQ get_desired_pose()
    {
        return xd_;
    }

    bool new_coppeliasim_xd_data_available()
    {
        return new_coppeliasim_xd_data_available_;
    }

    /*
    void publish_coppeliasim_b1_pose(const DQ& pose)
    {
        if (publisher_coppeliasim_b1_pose_)
        {
            geometry_msgs::msg::PoseStamped msg = sas::dq_to_geometry_msgs_pose_stamped(pose);
            publisher_coppeliasim_b1_pose_->publish(msg);
        }
    }
*/




};

void ControlExample::_update_kinematic_model()
{

}

ControlExample::~ControlExample()
{

}

/**
 * @brief ControlExample::ControlExample ctor of the class
 * @param node ROS2 node
 * @param configuration The configuration
 * @param break_loops The atomic boolean to break any loops when the user triggers the interruption signal
 */
ControlExample::ControlExample(std::shared_ptr<Node> &node,
                               const ControlExampleConfiguration &configuration,
                               std::atomic_bool *break_loops)
:
clock_{configuration.thread_sampling_time_sec},
configuration_{configuration},
st_break_loops_{break_loops},
node_{node},
robot_reached_region_{false}
{

}

/**
 * @brief ControlExample::control_loop
 */
void ControlExample::control_loop()
{

}


}
