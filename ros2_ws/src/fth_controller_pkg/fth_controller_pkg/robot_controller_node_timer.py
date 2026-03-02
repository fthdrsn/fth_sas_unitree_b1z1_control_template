#!/usr/bin/env python3
"""
SAS B1/Z1 Controller Node (Executor-Safe Version)
Timer-based control loop (no nested spinning)
"""

import yaml
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from robot_actions.action import MoveToGoal
from dqrobotics import *
from fth_controller_pkg.robot_model import B1Z1RobotModel
from scipy.spatial.transform import Rotation as R
from fth_controller_pkg.fth_robot_controller import RobotController
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import math


class ControllerNodeReconstruction(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        self.robot_pose = None
        self.joint_states = None
        self.safety_stop_active = False


        self.active_goal_handle = None
        self.target_pose_dq = None
        self.method_to_use = None
        self.goal_active = False
        self.task_error = float('inf')

      
        config_file_path = os.path.join(
            get_package_share_directory('fth_controller_pkg'),
            'config',
            'controller_config.yaml'
        )

        with open(config_file_path, 'r') as configFile:
            self.configs = yaml.safe_load(configFile)

        sim = self.configs.get('SimulationSettings', {})
        self.B1_topic_prefix = sim.get('B1_topic_prefix', 'sas_b1/b1_1')
        self.Z1_topic_prefix = sim.get('Z1_topic_prefix', 'sas_z1/z1_1')
        self.thread_sampling_time_sec = sim.get('thread_sampling_time_sec', 0.002)


        self.robot_model = B1Z1RobotModel()
        self.robot_controller = RobotController(self.configs, self.robot_model, {})

        self.create_subscription(
            PoseStamped,
            f'{self.B1_topic_prefix}/get/ekf/robot_pose',
            self.pose_callback,
            1
        )

        self.create_subscription(
            JointState,
            f'{self.Z1_topic_prefix}/get/joint_states',
            self.joint_states_callback,
            1
        )

        self.create_subscription(
            Bool,
            '/robot/safety_stop',
            self.safety_stop_callback,
            10
        )

    
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'{self.Z1_topic_prefix}/set/target_joint_positions',
            1
        )

        self.velocity_cmd_pub = self.create_publisher(
            Float64MultiArray,
            f'{self.B1_topic_prefix}/set/holonomic_target_velocities',
            1
        )


        self._action_server = ActionServer(
            self,
            MoveToGoal,
            'move_to_goal',
            self.execute_move_to_goal_callback
        )

        self.control_timer = self.create_timer(
            self.thread_sampling_time_sec,  # 100 Hz control loop
            self.control_loop_step
        )

        self.get_logger().info("Controller node initialized (Timer-based control).")


    def pose_callback(self, msg):
        self.robot_pose = msg

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def safety_stop_callback(self, msg):
        self.safety_stop_active = bool(msg.data)
        if self.safety_stop_active:
            self.get_logger().warn("Safety stop activated.")
            self.stop_robot_motion()

    def execute_move_to_goal_callback(self, goal_handle):

        self.get_logger().info("Received MoveToGoal request")

        # Wait until state available
        while self.robot_pose is None or self.joint_states is None:
            time.sleep(0.01)

        self.X_IMU = 1+0.5*E_*self.robot_pose.pose.position.z*k_
        self.robot_model.update_robot_model(self.X_J1_OFFSET, self.X_IMU, include_camera_transform=True)
        self.robot_controller.robot_model=self.robot_model

        self.target_pose_dq = self.posestamped_to_dq(
            goal_handle.request.goal_pose
        )

        self.method_to_use = goal_handle.request.method
        self.active_goal_handle = goal_handle
        self.goal_active = True
        self.task_error = float('inf')

        error_tol = self.configs["ControllerSettings"].get("errorTolerance", 0.01)

        # # Wait until goal finishes (timer handles control)
        while self.goal_active:
            time.sleep(0.01)

        result = MoveToGoal.Result()
        result.success = self.task_error <= error_tol
        result.final_error = float(self.task_error)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.active_goal_handle = None

        return result

    def control_loop_step(self):

        # if not self.goal_active:
        #     return
    
        print("Control loop step - active goal, computing control...")
        if self.robot_pose is None or self.joint_states is None:
            return

        if self.safety_stop_active:
            self.stop_robot_motion()
            self.goal_active = False
            return

        try:
            base_x = self.robot_pose.pose.position.x
            base_y = self.robot_pose.pose.position.y
            quat = self.robot_pose.pose.orientation
            yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]

            joint_positions = list(self.joint_states.position[:-1])
            robot_q = [base_x, base_y, yaw] + joint_positions

            method_map = {0: "FOCUS", 1: "SAMPLING", 2: "NOPATH"}
            method = method_map.get(self.method_to_use, "NOPATH")

            u, _, task_err = self.robot_controller.compute_one_step_u(
                self.target_pose_dq,
                robot_q,
                method=method
            )

            u = np.array(u).flatten()[:9]

            vel_lim_base = np.array(
                self.configs["RobotConstraints"].get("velLimBase", [0.15, 0.15, 0.15])
            )
            vel_lim_arm = np.array(
                self.configs["RobotConstraints"].get("velLimArm", [1.57]*6)
            )

            u[0:3] = np.clip(u[0:3], -vel_lim_base, vel_lim_base)
            u[3:9] = np.clip(u[3:9], -vel_lim_arm, vel_lim_arm)

            self.task_error = float(np.linalg.norm(task_err))

            # Publish base velocity
            base_vel = self.world_to_robot_frame(u[0:3], yaw)
            self.publish_velocity_commands(base_vel.tolist())

            # Publish joint targets
            joint_vel = u[3:9]
            target_joint_positions = (
                np.array(joint_positions)
                + joint_vel * self.thread_sampling_time_sec
            )

            self.publish_joint_commands(
                np.concatenate([
                    target_joint_positions,
                    [self.joint_states.position[-1]]
                ])
            )

            error_tol = self.configs["ControllerSettings"].get("errorTolerance", 0.01)

            if self.task_error <= error_tol:
                self.get_logger().info("Goal reached.")
                self.stop_robot_motion()
                self.goal_active = False

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
            self.stop_robot_motion()
            self.goal_active = False


    def stop_robot_motion(self):
        self.publish_velocity_commands([0.0, 0.0, 0.0])
        if self.joint_states is not None:
            self.publish_joint_commands(list(self.joint_states.position))

    def world_to_robot_frame(self, u_world, base_phi):
        rot_mat = np.array([
            [math.cos(base_phi), math.sin(base_phi), 0],
            [-math.sin(base_phi), math.cos(base_phi), 0],
            [0, 0, 1]
        ])
        return rot_mat @ u_world

    def publish_joint_commands(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_cmd_pub.publish(msg)

    def publish_velocity_commands(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_cmd_pub.publish(msg)

    def posestamped_to_dq(self, pose_msg):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        z = pose_msg.pose.position.z
        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w

        rotation = qw + qx*i_ + qy*j_ + qz*k_
        translation = x*i_ + y*j_ + z*k_
        return normalize(rotation + 0.5*E_*translation*rotation)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNodeReconstruction()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot_motion()
        node.destroy_node()



if __name__ == '__main__':
    main()