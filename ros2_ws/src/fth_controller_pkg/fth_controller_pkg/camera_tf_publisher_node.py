#!/usr/bin/env python3
"""
EEF Pose Publisher Node
Subscribes to joint states and robot pose
Computes and publishes end-effector pose using robot kinematic model
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from dqrobotics import *
from fth_controller_pkg.robot_model import B1Z1RobotModel
from scipy.spatial.transform import Rotation as R
import numpy as np


class CameraTFPublisherNode(Node):
    
    def __init__(self):
        super().__init__('camera_tf_publisher_node')
        
        # Robot state variables
        self.robot_pose = None
        self.joint_states = None
        self.robot_model = B1Z1RobotModel()
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Offset for the first joint of the arm w.r.t robot base frame
        X_J1_tra=2.96376995e-01*i_ -3.57461061e-17*j_+  1.61730000e-01*k_
        X_J1_rot=1.00000000e+00 -3.06272860e-20*i_ -3.15365499e-21*j_ -1.11022302e-16*k_
        self.X_J1_OFFSET =normalize(X_J1_rot+0.5*E_*X_J1_tra*X_J1_rot)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/sas_b1/b1_1/get/ekf/robot_pose',
            self.pose_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/sas_z1/z1_1/get/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Timer to publish at regular intervals
        self.create_timer(0.05, self.publish_camera_tf)
        
        self.get_logger().info('Camera TF Publisher Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /sas_b1/b1_1/get/ekf/robot_pose')
        self.get_logger().info('  - /sas_z1/z1_1/get/joint_states')
        self.get_logger().info('Publishing TF:')
        self.get_logger().info('  - world -> camera_link')
    
    def pose_callback(self, msg):
        """Callback for robot base pose from EKF"""
        self.robot_pose = msg
    
    def joint_states_callback(self, msg):
        """Callback for joint states"""
        self.joint_states = msg
    
    def publish_camera_tf(self):
        """Compute and publish camera pose as TF"""
        # Check if we have all necessary data
        if self.robot_pose is None or self.joint_states is None:
            return
        
        try:
            # Extract base pose
            base_x = self.robot_pose.pose.position.x
            base_y = self.robot_pose.pose.position.y
            X_IMU = 1+0.5*E_*self.robot_pose.pose.position.z*k_
            self.robot_model.update_robot_model(self.X_J1_OFFSET, X_IMU, include_camera_transform=True)
            quat = self.robot_pose.pose.orientation
            yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
            
            # Extract joint positions (exclude last joint if it's a gripper)
            joint_positions = list(self.joint_states.position[:-1])
            
            # Create full robot configuration [base_x, base_y, base_yaw, joint1, ..., joint6]
            robot_q = [base_x, base_y, yaw] + joint_positions
            
            # Compute forward kinematics to get camera pose
            camera_pose_dq = self.robot_model.Kinematics.fkm(robot_q)
            
            # Convert dual quaternion to TransformStamped
            transform_msg = self.dq_to_transform_stamped(camera_pose_dq)
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error computing camera pose: {e}')
    
    def dq_to_transform_stamped(self, dq, parent_frame='world', child_frame='camera_depth_optical_frame'):
        """Convert Dual Quaternion to TransformStamped"""
        trans = vec3(translation(dq))
        rot = vec4(rotation(dq))
        
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = parent_frame
        transform_msg.child_frame_id = child_frame
        
        transform_msg.transform.translation.x = trans[0]
        transform_msg.transform.translation.y = trans[1]
        transform_msg.transform.translation.z = trans[2]
        transform_msg.transform.rotation.w = rot[0]
        transform_msg.transform.rotation.x = rot[1]
        transform_msg.transform.rotation.y = rot[2]
        transform_msg.transform.rotation.z = rot[3]
        
        return transform_msg

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraTFPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')


if __name__ == '__main__':
    main()

