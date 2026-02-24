#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/utils/DQ_Math.h>
//#include "sas_consensus_control_unitree_b1z1/sas_consensus_control_unitree_b1z1.hpp"
#include "control_example/control_example.hpp"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("sas_consensus_control_node");

    try
    {

        sas::ControlExampleConfiguration configuration;
        sas::get_ros_parameter(node,"cs_host",configuration.cs_host);
        sas::get_ros_parameter(node,"cs_port",configuration.cs_port);
        sas::get_ros_parameter(node,"cs_TIMEOUT_IN_MILISECONDS",configuration.cs_TIMEOUT_IN_MILISECONDS);
        sas::get_ros_parameter(node,"cs_B1_robotname",configuration.cs_B1_robotname);
        sas::get_ros_parameter(node,"cs_Z1_robotname",configuration.cs_Z1_robotname);
        sas::get_ros_parameter(node,"B1_topic_prefix",configuration.B1_topic_prefix);
        sas::get_ros_parameter(node,"Z1_topic_prefix",configuration.Z1_topic_prefix);
        sas::get_ros_parameter(node, "cs_desired_frame", configuration.cs_desired_frame);
        sas::get_ros_parameter(node,"thread_sampling_time_sec",configuration.thread_sampling_time_sec);

        std::vector<double> configuration_limits_min;
        std::vector<double> configuration_limits_max;
        sas::get_ros_parameter(node,"configuration_limits_min_deg", configuration_limits_min);
        sas::get_ros_parameter(node,"configuration_limits_max_deg", configuration_limits_max);
        configuration.configuration_limits = {deg2rad(sas::std_vector_double_to_vectorxd(configuration_limits_min)),
                                              deg2rad(sas::std_vector_double_to_vectorxd(configuration_limits_max))};


        std::vector<double> config_vel_limits_min;
        std::vector<double> config_vel_limits_max;
        sas::get_ros_parameter(node,"config_vel_limits_min", config_vel_limits_min);
        sas::get_ros_parameter(node,"config_vel_limits_max", config_vel_limits_max);
        configuration.configuration_vel_limits = {sas::std_vector_double_to_vectorxd(config_vel_limits_min),
                                                  sas::std_vector_double_to_vectorxd(config_vel_limits_max)};

        auto driver = std::make_shared<sas::ControlExample>(node,configuration, &kill_this_process);
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");
        driver->control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;


}
