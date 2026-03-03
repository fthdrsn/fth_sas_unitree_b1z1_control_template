#!/usr/bin/env python3
"""
SAS B1/Z1 Controller Node
Subscribes to robot pose and joint states
Publishes target joint positions and holonomic velocities
"""

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, ColorRGBA, Bool, Float64
from dqrobotics import *
from fth_controller_pkg.robot_model import B1Z1RobotModel
from scipy.spatial.transform import Rotation as R
from fth_controller_pkg.fth_robot_controller import RobotController
from ament_index_python.packages import get_package_share_directory
from robot_actions.msg import TargetPose, GoalState
import os
import numpy as np
import math
import time
import signal
class ControllerNodeReconstruction(Node):
    
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Robot state variables
        self.robot_pose = None
        self.joint_states = None        
        self.safety_stop_active = False
        
        # Action server state
        self.active_goal_handle = None
        self.robot_target={"id": None, "target_pose": None, "target_pose_dq": None}
        self.focus_point = [0.0, 0.0, 0.0]
        
        # Load controller config
        # Get parameters
        config_file_path= os.path.join(
            get_package_share_directory('fth_controller_pkg'),
            'config',
            'controller_config.yaml'
        )

        with open(config_file_path, 'r') as configFile:
            controller_configs = yaml.safe_load(configFile)

        self.configs=controller_configs

        self.simulation_settings = self.configs.get('SimulationSettings', {})
        self.cs_host = self.simulation_settings.get('cs_host', 'localhost')
        self.cs_port = self.simulation_settings.get('cs_port', 23000)
        self.cs_TIMEOUT_IN_MILISECONDS = self.simulation_settings.get('cs_TIMEOUT_IN_MILISECONDS', 5000)
        self.B1_robotname = self.simulation_settings.get('cs_B1_robotname', 'UnitreeB1_1')
        self.Z1_robotname = self.simulation_settings.get('cs_Z1_robotname', 'UnitreeZ1')
        self.cs_desired_frame = self.simulation_settings.get('cs_desired_frame', 'xd1')
        self.B1_topic_prefix = self.simulation_settings.get('B1_topic_prefix', 'sas_b1/b1_1')
        self.Z1_topic_prefix = self.simulation_settings.get('Z1_topic_prefix', 'sas_z1/z1_1')
        self.thread_sampling_time_sec = self.simulation_settings.get('thread_sampling_time_sec', 0.002)
        self.search_space_radius = self.simulation_settings.get('search_space_radius', 0.5)
        self.search_space_center= self.simulation_settings.get('search_space_center', [0.0, 0.0, 0.0])

        
        # Get pose of workspace planes w.r.t world frame
        plane_0_tra=0.033*i_-2.15425013*j_+0.5*k_
        plane_0_ori=7.07106781e-01-7.07106781e-01*i_-2.22044605e-16*j_+ 0.0*k_
        self.plane_0_pose_dq = normalize(plane_0_ori+0.5*E_*plane_0_tra*plane_0_ori)
        
        plane_2_tra=2.73141391*i_-0.105*j_+0.5*k_
        plane_2_ori=-0.5+0.5*i_+0.5*j_-0.5*k_
        self.plane_2_pose_dq = normalize(plane_2_ori+0.5*E_*plane_2_tra*plane_2_ori)
        
        plane_3_tra=-2.73250527*i_-0.183*j_+0.5*k_
        plane_3_ori=-0.5+0.5*i_-0.5*j_+0.5*k_
        self.plane_3_pose_dq = normalize(plane_3_ori+0.5*E_*plane_3_tra*plane_3_ori)
        
        plane_4_tra=-0.083*i_+1.84929974*j_+0.5*k_
        plane_4_ori=5.37347944e-14-7.33302308e-14*i_ +7.07106781e-01*j_-7.07106781e-01*k_
        self.plane_4_pose_dq = normalize(plane_4_ori+0.5*E_*plane_4_tra*plane_4_ori)

       
        b1_front_bumper_tra=0.36128224*i_ -0.00080654*j_ -0.09314052*k_
        b1_front_bumper_ori=1-6.16297582e-33*i_-3.29597460e-17*j_+0.0*k_
        self.b1_front_bumper_pose_dq = normalize(b1_front_bumper_ori+0.5*E_*b1_front_bumper_tra*b1_front_bumper_ori)
        
        b1_center_bumper_tra=-0.05067116*i_ -0.00476247*j_ -0.13453465*k_
        b1_center_bumper_ori=1.00000000e+00 -8.67361738e-19*i_ -3.38271078e-17*j_+ 0.0*k_
        self.b1_center_bumper_pose_dq = normalize(b1_center_bumper_ori+0.5*E_*b1_center_bumper_tra*b1_center_bumper_ori)
        
        b1_rear_bumper_tra=-0.50503677*i_ -0.02950913*j_ -0.14559503*k_
        b1_rear_bumper_ori= 1.00000000e+00 + 3.08148791e-33*i_ -3.29597460e-17*j_+  0.0*k_
        self.b1_rear_bumper_pose_dq = normalize(b1_rear_bumper_ori+0.5*E_*b1_rear_bumper_tra*b1_rear_bumper_ori)
       

        self.const_pose_dict = {
            'workspace_planes':
            {
            'plane_0_pose_dq': [self.plane_0_pose_dq,0.0,1.0], #Planes poses w.r.t world frame, and offset
            'plane_2_pose_dq': [self.plane_2_pose_dq,0.0,1.0],
            'plane_3_pose_dq': [self.plane_3_pose_dq,0.0,1.0],
            'plane_4_pose_dq': [self.plane_4_pose_dq,0.0,1.0],
            },

            'robot_bumper_spheres':
            {
                'front_bumper_pose_dq': [self.b1_front_bumper_pose_dq, 0.3, 1.0], #Spheres poses w.r.t robot base frame, and radius
                'center_bumper_pose_dq': [self.b1_center_bumper_pose_dq, 0.3, 1.0],
                'rear_bumper_pose_dq': [self.b1_rear_bumper_pose_dq, 0.3, 1.0],
            },

            'workspace_cylinders':
            {
                'search_space_pose_dq': [1+0.5*E_*(self.search_space_center[0]*i_+self.search_space_center[1]*j_+self.search_space_center[2]*k_), self.search_space_radius,1.0], #Cylinder pose w.r.t world frame, and radius
            }
        }  
       
        X_J1_OFFSET_tra=2.96376995e-01*i_ -3.57461061e-17*j_+  1.61730000e-01*k_
        X_J1_OFFSET_ori= 1.00000000e+00 -3.06272860e-20*i_ -3.15365499e-21*j_ -1.11022302e-16*k_
        self.X_J1_OFFSET = normalize(X_J1_OFFSET_ori+0.5*E_*X_J1_OFFSET_tra*X_J1_OFFSET_ori) #Pose of the first joint of the arm w.r.t robot base frame

        self.robot_model = B1Z1RobotModel()        
        self.robot_controller = RobotController(controller_configs, self.robot_model, self.const_pose_dict)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
           f'{self.B1_topic_prefix}/get/ekf/robot_pose',
            self.pose_callback,
            1
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            f'{self.Z1_topic_prefix}/get/joint_states',
            self.joint_states_callback,
            1
        )

        self.focus_point_sub = self.create_subscription(
            Float64MultiArray,
            '/nbv/focus_point',
            self.focus_point_callback,
            10
        )

        self.safety_stop_sub = self.create_subscription(
            Bool,
            '/robot/safety_stop',
            self.safety_stop_callback,
            10
        )
        
        # Publishers
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

        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            f'{self.B1_topic_prefix}/set/coppeliasim_frame_x',
            1
        )
        
        self.target_sub = self.create_subscription(TargetPose, '/controller/target_nbv', self.target_pose_callback, 10)
        self.state_pub = self.create_publisher(GoalState, '/controller/goal_state', 10)

        self.new_pose_received = False
        self.is_model_updated = False
        self.tick_count = 0
        self.total_step_time = 0.0

        self.get_logger().info('Controller Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info(f'  - {self.B1_topic_prefix}/get/ekf/robot_pose')
        self.get_logger().info(f'  - {self.Z1_topic_prefix}/get/joint_states')
        self.get_logger().info('Publishing to:')
        self.get_logger().info(f'  - {self.Z1_topic_prefix}/set/target_joint_positions')
        self.get_logger().info(f'  - {self.B1_topic_prefix}/set/holonomic_target_velocities')
        self.get_logger().info(f'  - /robot/safety_stop')
       
        

    def target_pose_callback(self, msg):
        """Callback for receiving target NBV pose"""
        target_pose = msg.target_pose
        self.robot_target["id"] = msg.id
        self.robot_target["target_pose"] = target_pose
        self.robot_target["target_pose_dq"] = self.posestamped_to_dq(target_pose)
        self.new_pose_received = True
        self.get_logger().info('Received target NBV pose, ready to execute goal.')
    

    def control_step(self):
        step_start = time.monotonic()
        
        if self.safety_stop_active:
            # ensure zero commands while safety stop active
            self.publish_velocity_commands([0.0, 0.0, 0.0])
            return
        
        if not self.new_pose_received or self.joint_states is None or self.robot_pose is None:
            return
        
        # Update the robot model
        if self.is_model_updated == False:
            self.X_IMU = 1+0.5*E_*self.robot_pose.pose.position.z*k_
            self.robot_model.update_robot_model(self.X_J1_OFFSET, self.X_IMU, include_camera_transform=True)
            self.robot_controller.robot_model=self.robot_model
            self.is_model_updated = True
        
        base_x = self.robot_pose.pose.position.x
        base_y = self.robot_pose.pose.position.y
        quat = self.robot_pose.pose.orientation
        yaw = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]

        joint_positions = list(self.joint_states.position[:-1])
        robot_q = [base_x, base_y, yaw] + joint_positions
        
        try:
            self.robot_controller.constraint_switches["enableVisibilityConst"] = True
            u, _, task_err = self.robot_controller.compute_one_step_u(
                    self.robot_target["target_pose_dq"],
                    robot_q,
                    focus_point=[0,0,1],
                    method="SAMPLING"
                )
        except Exception as e:
            self.get_logger().error(f'Error computing control step: {e}')
            self.publish_velocity_commands([0.0, 0.0, 0.0])
            out = GoalState()
            out.id = self.robot_target["id"]
            out.is_done = True
            out.is_success = False
            out.is_error = True
            out.is_stability = False
            self.state_pub.publish(out)
            return
            
        u = np.array(u).flatten()
        # When the visibility soft constraint u includes slack variable
        if u.shape[0] > 9:
            u = u[:9]

            
        task_error = float(np.linalg.norm(task_err))

        base_vel = u[0:3]
        world_to_robot_vel = self.world_to_robot_frame(base_vel, yaw)
        joint_vel = u[3:9]
        target_joint_positions = (np.array(joint_positions) + joint_vel * self.thread_sampling_time_sec).tolist()

        current_eef_pose_dq = self.robot_model.Kinematics.fkm(robot_q)

        self.publish_joint_commands(np.concatenate([target_joint_positions, np.array([self.joint_states.position[-1]])]))
        self.publish_velocity_commands(world_to_robot_vel.tolist())
        
        current_pose = self.dq_to_posestamped(current_eef_pose_dq)                    
        self.current_pose_pub.publish(current_pose)
        if task_error < self.configs["ControllerSettings"].get("errorTolerance", 0.01):
            self.get_logger().info('Goal reached with task error: {:.4f}'.format(task_error))
            self.publish_velocity_commands([0.0, 0.0, 0.0])
            self.new_pose_received = False
            out = GoalState()
            out.id = self.robot_target["id"]
            out.is_done = True
            out.is_success = True
            out.is_error = False
            out.is_stability = False
            self.state_pub.publish(out)
        else:
            #self.get_logger().info('Current task error: {:.4f}'.format(task_error))
            out = GoalState()
            out.id = self.robot_target["id"]
            out.is_done = False
            out.is_success = False
            out.is_error = False
            out.is_stability = False
            self.state_pub.publish(out)
        
        # Timing instrumentation
        step_elapsed = time.monotonic() - step_start
        self.total_step_time += step_elapsed
        self.tick_count += 1
        if self.tick_count % 100 == 0:
            avg_time_ms = (self.total_step_time / self.tick_count) * 1000
            period_ms = self.thread_sampling_time_sec * 1000
            self.get_logger().info(f'Avg control_step: {avg_time_ms:.2f}ms (budget: {period_ms:.2f}ms)')
            if avg_time_ms > period_ms * 0.8:
                self.get_logger().warn(f'⚠️  Using >80% of time budget! May cause timing issues.')

    def safety_stop_callback(self, msg):
        """Callback for topic-driven safety stop (latched)."""
        requested_state = bool(msg.data)
        if requested_state == self.safety_stop_active:
            return
        self.safety_stop_active = requested_state

    def pose_callback(self, msg):
        """Callback for robot pose from EKF"""
        self.robot_pose = msg

    def joint_states_callback(self, msg):
        """Callback for joint states"""
        self.joint_states = msg

    def focus_point_callback(self, msg):
        """Callback for focus point updates"""
        if msg.data and len(msg.data) >= 3:
            self.focus_point = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]

        
    def posestamped_to_dq(self, pose_msg):
        """Convert PoseStamped to Dual Quaternion"""
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        z = pose_msg.pose.position.z
        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w
        
        rotation = qw+qx*i_+qy*j_+qz*k_
        translation = x*i_+y*j_+z*k_
        return normalize(rotation + 0.5*E_*translation*rotation)
    
    def dq_to_posestamped(self, dq, frame_id='world'):
        """Convert Dual Quaternion to PoseStamped"""
        trans = vec3(translation(dq))
        rot =vec4(rotation(dq))  
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = trans[0]
        pose_msg.pose.position.y = trans[1]
        pose_msg.pose.position.z = trans[2]
        pose_msg.pose.orientation.w = rot[0]
        pose_msg.pose.orientation.x = rot[1]
        pose_msg.pose.orientation.y = rot[2]
        pose_msg.pose.orientation.z = rot[3]
        return pose_msg
    
    def world_to_robot_frame(self, u_world, base_phi):

        rot_mat = np.array([[math.cos(base_phi), math.sin(base_phi), 0],
                                [-math.sin(base_phi),
                                math.cos(base_phi), 0],
                                [0, 0, 1]])
        base_vel_ = rot_mat@u_world

        return base_vel_


    def publish_joint_commands(self, positions):
        """Publish target joint positions"""
        msg = Float64MultiArray()
        msg.data = positions
        self.joint_cmd_pub.publish(msg)
    
    def publish_velocity_commands(self, velocities):
        """Publish holonomic velocities"""

        msg = Float64MultiArray()
        msg.data = velocities
        self.velocity_cmd_pub.publish(msg)
    

def main(args=None):

    rclpy.init(args=args)
    node = ControllerNodeReconstruction()

    def stop_robot(signum, frame):
        node.get_logger().info('Received shutdown signal, stopping robot...')
        node.publish_velocity_commands([0.0, 0.0, 0.0])
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)

    try:
        rate_hz = 1.0 / node.thread_sampling_time_sec
        period = node.thread_sampling_time_sec
        next_tick = time.monotonic() + period
        safety_margin = period * 0.0 # Leave 10% headroom for spin_once overhead

        while rclpy.ok():
            # spin for callbacks with safety margin
            now = time.monotonic()
            to_wait = max(0.0, next_tick - now - safety_margin)
            rclpy.spin_once(node, timeout_sec=to_wait)

            now = time.monotonic()
            if now >= next_tick:
                tick_start = now
                node.control_step()
                next_tick += period
                # warn if overrun
                step_duration = time.monotonic() - tick_start
                if step_duration > period:
                    node.get_logger().warning(f'control step exceeded period: {step_duration*1000:.2f}ms > {period*1000:.2f}ms')
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled excepts", e)
   


if __name__ == '__main__':
    main()