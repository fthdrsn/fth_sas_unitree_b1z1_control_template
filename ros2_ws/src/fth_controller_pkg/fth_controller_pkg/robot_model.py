from math import pi, sin, cos
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH, DQ_HolonomicBase, DQ_SerialWholeBody
import math
import numpy as np
from typing import List

class B1Z1RobotModel():
    """ The robot model of the B1Z1 mobile manipulator. It includes the kinematic model of the mobile base and the arm, as well as the transformation from the base to the camera."""
    def __init__(self):
        # DH Parameters of the Youbot arm
        self.dh_mat = np.array([[0,0,pi/2+1.3151, 0.2557, -pi/2,  -pi/2],
                               [0.045, 0,0,0,0, 0.0492],
                               [ 0,  -0.35,-0.22533,-0.07,0, 0],
                               [-pi/2, 0,0,-pi/2,pi/2,0],
                               [0, 0, 0, 0, 0,0]])
        
        self.end_effector_transform= 1 + 0.5*E_*0.2*k_
        T_cam =-0.05* i_
        quat_cam = cos(pi/4) + sin(pi/4)*k_
        self.camera_transform = quat_cam + 0.5*E_*T_cam*quat_cam
    
        

    def update_robot_model(self, X_J1_OFFSET, X_IMU, include_camera_transform: bool = True):
       
        # Create the kinematic model of mobile maniopulator
        self.X_J1_OFFSET = X_J1_OFFSET
        X_IMU_np=vec3(translation(X_IMU))
        self.X_HEIGHT_OFFSET_ =1+0.5*E_*X_IMU_np[2]*k_

        arm = DQ_SerialManipulatorDH(self.dh_mat)
        arm.set_base_frame(DQ([1]))
        arm.set_reference_frame(self.X_J1_OFFSET)
      
        if include_camera_transform:
            last_frame_transform= normalize(self.end_effector_transform*self.camera_transform)
        else:
            last_frame_transform=self.end_effector_transform
        
        arm.set_effector(last_frame_transform)
        base = DQ_HolonomicBase()
        base.set_frame_displacement(self.X_HEIGHT_OFFSET_ )
        kin = DQ_SerialWholeBody(base)
        kin.add(arm)
        self.b1z1_kinematic = kin
    

    @property
    def Kinematics(self):
        """ Get the kinematic model of the B1Z1 mobile manipulator.
        :return: kinematic model of the B1Z1 mobile manipulator
        """
        return self.b1z1_kinematic

    @property
    def BaseKinematics(self):
        """ Get the kinematic model of the holonomic base.
        :return: kinematic model of the holonomic base
        """
        return self.b1z1_kinematic.get_chain_as_holonomic_base(0)

    @property
    def ArmKinematics(self):
        """ Get the kinematic model of the arm.
        :return: kinematic model of the arm
        """
        return self.b1z1_kinematic.get_chain_as_serial_manipulator_dh(1)
