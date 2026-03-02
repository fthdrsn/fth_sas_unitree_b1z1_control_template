#!/usr/bin/env python3
"""
Launch file for controller and EEF pose publisher
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description with controller and publisher nodes"""
    
    # Path to virtual environment
    venv_path = '/app/software/virtual_env/'
    #venv_path = '/home/laptop13/ros2_tutorial_workspace/src/dog_env/'
    
    # Get the package share directory for RViz config
    pkg_share_dir = get_package_share_directory('fth_controller_pkg')
    rviz_config_path = os.path.join(pkg_share_dir, 'config', 'reconstruction_rviz.rviz')
    
    # Set environment to use virtual environment
    set_python_path = SetEnvironmentVariable(
        'PYTHONPATH',
        os.path.join(venv_path, 'lib', 'python3.12', 'site-packages') + ':' + 
        os.environ.get('PYTHONPATH', '')
    )
    
    # Robot controller node
    robot_controller_node = Node(
        package='fth_controller_pkg',
        executable='robot_controller',
        name='robot_controller_node',
        output='screen'
    )

    controller_test_node = Node(
        package='fth_controller_pkg',
        executable='controller_test',
        name='controller_test_node',
        output='screen'
    )
    
    # EEF pose publisher node
    camera_tf_publisher_node = Node(
        package='fth_controller_pkg',
        executable='camera_tf_publisher',
        name='camera_tf_publisher_node',
        output='screen'
    )
    
    
    return LaunchDescription([
        set_python_path,
        #controller_test_node,
        robot_controller_node,
        camera_tf_publisher_node,
        #rviz_node,
    ])

