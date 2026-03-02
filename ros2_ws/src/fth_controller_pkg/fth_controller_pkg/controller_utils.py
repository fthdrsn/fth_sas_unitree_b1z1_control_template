from scipy.spatial import cKDTree
from dqrobotics import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from dqrobotics.utils import DQ_Geometry
import math
pi2 = math.pi/2


def compute_bumpers_to_workspace_constraints(constaint_pose_dic, robot_model, robot_q):

    """Compute the constraints between the bumper spheres on the robot and the planes and cylinders in the workspace.
    Args:        constaint_pose_dic: A dictionary containing the poses of the planes and cylinders w.r.t world frame and the bumper spheres w.r.t robot base.
        robot_model: The kinematic model of the robot.
        robot_q: The joint values of the robot.
    Returns:        A tuple containing the Jacobian of the constraints and the corresponding b vector.
    """

    robot_base_kinematics = robot_model.BaseKinematics # Base kinematics including z offset
    robot_base_pose = robot_base_kinematics.fkm(robot_q)
    robot_base_Jacobian = robot_base_kinematics.pose_jacobian(robot_q, 2)

    constaint_J = []
    constaint_b = []

    for robot_bumper in constaint_pose_dic["robot_bumper_spheres"].values():
        bumper_pose_dq = robot_bumper[0] # Pose of the bumper sphere w.r.t robot base frame
        bumper_rad = robot_bumper[1] #Radius of the bumper sphere

        bumper_pose_world_dq=robot_base_pose*bumper_pose_dq #Pose of the bumper sphere w.r.t world frame
        bumper_pose_J=haminus8(bumper_pose_dq)@robot_base_Jacobian # Pose Jacobian of the bumper 
        bumber_transition_J=robot_model.Kinematics.translation_jacobian(bumper_pose_J, bumper_pose_world_dq) # Transition Jacobian of the bumper
        bumber_transition_J = np.concatenate((bumber_transition_J, np.zeros((4, 6))), axis=1) # Expand with zero for arm joints
        
        for plane in constaint_pose_dic["workspace_planes"].values():
            plane_pose_dq = plane[0]
            plane_dq=pose_to_plane(plane_pose_dq, k_)
            plane_offsett = plane[1]
            lambda_ = plane[2]
            dist_J_p2plane=robot_model.Kinematics.point_to_plane_distance_jacobian(bumber_transition_J, translation(bumper_pose_world_dq), plane_dq)
            dist_p2plane = DQ_Geometry.point_to_plane_distance(translation(bumper_pose_world_dq), plane_dq) - (bumper_rad + plane_offsett)
            constaint_J.append(-dist_J_p2plane)
            constaint_b.append(lambda_*dist_p2plane)

        for cylinder in constaint_pose_dic["workspace_cylinders"].values():
            cylinder_pose_dq = cylinder[0] # Pose of the cylinder w.r.t world frame
            cylinder_rad = cylinder[1] # Radius of the cylinder
            lambda_ = cylinder[2]

            cyl_line_dq=pose_to_line(cylinder_pose_dq, k_)
            dist_J_p2cyl=robot_model.Kinematics.point_to_line_distance_jacobian(bumber_transition_J,translation(bumper_pose_world_dq), cyl_line_dq)
         
            dist_p2cyl = DQ_Geometry.point_to_line_squared_distance(translation(bumper_pose_world_dq), cyl_line_dq) - (bumper_rad + cylinder_rad)**2
            constaint_J.append(-dist_J_p2cyl)
            constaint_b.append(lambda_*dist_p2cyl)

    return np.concatenate(constaint_J, axis=0), np.array(constaint_b)

def compute_eef_to_workspace_constraints(constaint_pose_dic, robot_model, robot_q):
    
    robot_kinematics = robot_model.Kinematics # Base kinematics including z offset
    robot_pose = robot_kinematics.fkm(robot_q)
    robot_pose_J = robot_kinematics.pose_jacobian(robot_q)
    robot_transition_J = robot_kinematics.translation_jacobian(robot_pose_J, robot_pose)
    eef_rad=0.075
    constaint_J = []
    constaint_b = []

    for plane in constaint_pose_dic["workspace_planes"].values():
        plane_pose_dq = plane[0]
        plane_dq=pose_to_plane(plane_pose_dq, k_)
        plane_offsett = plane[1]
        lambda_ = plane[2]
        dist_J_p2plane=robot_model.Kinematics.point_to_plane_distance_jacobian(robot_transition_J, translation(robot_pose), plane_dq)
        dist_p2plane = DQ_Geometry.point_to_plane_distance(translation(robot_pose), plane_dq) - (eef_rad + plane_offsett)
        constaint_J.append(-dist_J_p2plane)
        constaint_b.append(lambda_*dist_p2plane)
    for cylinder in constaint_pose_dic["workspace_cylinders"].values():
        cylinder_pose_dq = cylinder[0] # Pose of the cylinder w.r.t world frame
        cylinder_rad = cylinder[1] # Radius of the cylinder
        lambda_ = cylinder[2]
        cyl_line_dq=pose_to_line(cylinder_pose_dq, k_)

        dist_J_p2cyl=robot_model.Kinematics.point_to_line_distance_jacobian(robot_transition_J, translation(robot_pose), cyl_line_dq)
        
        dist_p2cyl = DQ_Geometry.point_to_line_squared_distance(translation(robot_pose), cyl_line_dq) - (eef_rad + cylinder_rad)**2
        constaint_J.append(-dist_J_p2cyl)
        constaint_b.append(lambda_*dist_p2cyl)

    return np.concatenate(constaint_J, axis=0), np.array(constaint_b)


   

def compute_base_constraints(pose_list, radius_list, robot_kin, robot_q):
    """Compute the base constraints for the YouBot robot using point-to-line distance.
    Args:
        pose_list: List of poses of the obstacles as dual quaternions.
        radius_list: List of radii of the obstacles.
        robot_kin: The robot kinematic model.
        youbot_q: Joint values of the YouBot robot.
    Returns:
        A tuple containing the Jacobian of the base constraints and the corresponding b vector.
    """
    youbot_q = np.array(youbot_q)
    youbot_base = robot_kin.get_chain_as_holonomic_base(0)
    # Do not take into account frame_displacement where arm connected(find fkm w.r.t ref frame of base)
    youbot_base_pose = youbot_base.raw_fkm(youbot_q)
    # fkm =raw_fkm(q)*frame_displacement
    # pose_jacob=haminus8(frame_displacement_)*raw_pose_jacob
    # Do not take into account frame_displacement where arm connected(find jacob w.r.t ref frame of base)
    youbot__base_Jx = youbot_base.raw_pose_jacobian(youbot_q, 2)

    t_inner = translation(youbot_base_pose)
    base_jt = robot_kin.translation_jacobian(
        youbot__base_Jx, youbot_base_pose)
    Jt = np.concatenate((base_jt, np.zeros((4, 5))), axis=1)
    b_constraint = []
    eta = 1
    idx = 0
    for pose_, rad_ in zip(pose_list, radius_list):

        j_dist = robot_kin.point_to_line_distance_jacobian(
            Jt, t_inner, Ad(pose_, k_))
        dist = DQ_Geometry.point_to_line_squared_distance(
            t_inner, Ad(pose_, k_)) - (rad_ + 0.35) ** 2

        if idx == 0:
            j_constraint = np.array(j_dist)
        else:
            j_constraint = np.concatenate((j_constraint, j_dist), axis=0)
        b_constraint.append(eta*dist)
        idx += 1

    return j_constraint, np.array(b_constraint)


def compute_base_constraints_p2p(pose_list, radius_list, robot_kin, youbot_q):
    """Compute the base constraints for the YouBot robot using point-to-point distance.
    Args:
        pose_list: List of poses of the obstacles as dual quaternions.
        radius_list: List of radii of the obstacles.
        robot_kin: The robot kinematic model.
        youbot_q: Joint values of the YouBot robot.
    Returns:
        A tuple containing the Jacobian of the base constraints and the corresponding b vector.
    """

    youbot_q = np.array(youbot_q)

    youbot_base = robot_kin.get_chain_as_holonomic_base(0)
    # Do not take into account frame_displacement where arm connected(find fkm w.r.t ref frame of base)
    youbot_base_pose = youbot_base.raw_fkm(youbot_q)
    # fkm =raw_fkm(q)*frame_displacement
    # pose_jacob=haminus8(frame_displacement_)*raw_pose_jacob
    # Do not take into account frame_displacement where arm connected(find jacob w.r.t ref frame of base)
    youbot__base_Jx = youbot_base.raw_pose_jacobian(youbot_q, 2)

    t_inner = translation(youbot_base_pose)
    base_jt = robot_kin.translation_jacobian(
        youbot__base_Jx, youbot_base_pose)
    Jt = np.concatenate((base_jt, np.zeros((4, 6))), axis=1)
    b_constraint = []
    eta = 1
    idx = 0
    for pose_, rad_ in zip(pose_list, radius_list):

        j_dist = robot_kin.point_to_point_distance_jacobian(
            Jt, t_inner, translation(pose_))
        dist = DQ_Geometry.point_to_point_squared_distance(
            t_inner, translation(pose_)) - (rad_ + 0.35) ** 2

        if idx == 0:
            j_constraint = np.array(j_dist)
        else:
            j_constraint = np.concatenate((j_constraint, j_dist), axis=0)
        b_constraint.append(eta*dist)
        idx += 1

    return j_constraint, np.array(b_constraint)

def compute_eef_constraints(pose_list, radius_list, robot_kin, youbot_q):
    """Compute the end-effector constraints for the YouBot robot.
    Args:
        pose_list: List of poses of the obstacles as dual quaternions.
        radius_list: List of radii of the obstacles.
        robot_kin: The robot kinematic model.
        youbot_q: Joint values of the YouBot robot.
    Returns:
        A tuple containing the Jacobian of the end-effector constraints and the corresponding b vector.
    """

    # Compute eef collision constraint with search space cylinder (point to line jacobian)
    youbot_q = np.array(youbot_q)
    cylinder_dq = pose_to_line(pose_list[0], k_)
    pose_ = robot_kin.fkm(youbot_q)
    position_eef = translation(pose_)
    pose_J = robot_kin.pose_jacobian(youbot_q)
    tra_J = robot_kin.translation_jacobian(
        pose_J, pose_)
    J_eef_cyl = robot_kin.point_to_line_distance_jacobian(
        tra_J, position_eef, cylinder_dq)

    b_eef_cyl = DQ_Geometry.point_to_line_squared_distance(
        position_eef, cylinder_dq) - radius_list[0]**2
    # Compute constraints between eef and work space obstacles(we model obtsancles as sphere since the line is infinite)
    b_constraint = []
    eta = 1
    idx = 0
    for pose_, rad_ in zip(pose_list[1:], radius_list[1:]):

        j_dist = robot_kin.point_to_point_distance_jacobian(
            tra_J, position_eef, translation(pose_))
        dist = DQ_Geometry.point_to_point_squared_distance(
            position_eef, translation(pose_)) - (rad_+0.05) ** 2

        if idx == 0:
            j_constraint = np.array(j_dist)
        else:
            j_constraint = np.concatenate((j_constraint, j_dist), axis=0)
        b_constraint.append(eta*dist)
        idx += 1
    j_constraint = np.concatenate((j_constraint, J_eef_cyl), axis=0)
    b_constraint.append(eta*b_eef_cyl)
    return j_constraint, np.array(b_constraint)

def pose_to_line(line_pose, direction):
    """Convert a pose to a line represented as a dual quaternion.
    Args:
        line_pose: A dual quaternion representing the pose of the line.
        direction: A 3D direction vector as a numpy array or list.
    Returns:
        A dual quaternion representing the line.
    """
    p = translation(line_pose)
    r = rotation(line_pose)
    l = Ad(r, direction)
    m = cross(p, l)
    return l + E_ * m

def get_direction(line_pose, direction):
    """Extract the direction vector from a pose represented as a dual quaternion.
    Args:
        line_pose: A dual quaternion representing the pose of the line.
        direction: A 3D direction vector as a numpy array or list.
    Returns:
        A dual quaternion representing the direction of the line.
    """
    p = translation(line_pose)
    r = rotation(line_pose)
    l = Ad(r, direction)

    return l

def pose_to_plane(plane_pose, normal):
    """Convert a pose to a plane represented as a dual quaternion.
    Args:
        plane_pose: A dual quaternion representing the pose of the plane.
        normal: A 3D normal vector as a numpy array or list.
    Returns:
        A dual quaternion representing the plane.
    """
    p = translation(plane_pose)
    r = rotation(plane_pose)
    n = Ad(r, normal)
    d = dot(p, n)
    return n + E_ * d

def direction_to_orientation(direction):
    """Convert a direction vector to a pose represented as a dual quaternion.
    Args:
        direction: A 3D direction vector as a numpy array or list.
    Returns:
        A dual quaternion representing the pose with the direction as the z-axis.
    """

    # Normalize the direction vector
    direction /= np.linalg.norm(direction)

    # Calculate the y-axis as the cross product of the direction vector and the [0, 1, 0] vector
    right = np.cross(np.array([0, 0, 1]), direction)
    right /= np.linalg.norm(right)

    # Calculate the x-axis as the cross product of the up vector and the direction vector
    y_vec = np.cross(direction, right)
    y_vec /= np.linalg.norm(y_vec)
    # Create the transformation matrix
    pose = np.eye(3)

    pose[:, 0] = right
    pose[:, 1] = y_vec
    pose[:, 2] = direction
    quaternion = R.from_matrix(pose).as_quat()  
    return quaternion[[3, 0, 1, 2]]  # Return in (w, x, y, z) order

def generate_points_on_cylinder(theta, cly_centre=(0, 0, 0), z=0.2, r=0.7):
    """Generate a point on a cylinder defined by its centre, radius, and height.
    Args:
        theta: Angle in radians to define the point's position on the cylinder.
        cly_centre: Centre of the cylinder as a tuple (x, y, z).
        z: Height of the cylinder.
        r: Radius of the cylinder.
    Returns:
        A point on the cylinder as a dual quaternion.
    """
    x = r*math.cos(theta)
    y = r*math.sin(theta)

    pose_point = (1+E_*0.5*(x*i_+y*j_+z*k_)) * \
        (1+E_*0.5*(cly_centre[0]*i_+cly_centre[1]*j_+cly_centre[2]*k_))

    return pose_point


def transform_to_image_plane(pnt, im_width=256, im_height=256, cam_fov=57):
    """Transform a point in the image plane to the camera's field of view.
    Args:
        pnt: A 2D point in the image plane (x, y).
        im_width: Width of the image in pixels.
        im_height: Height of the image in pixels.
        cam_fov: Camera field of view in degrees.
    Returns:
        A 3D point in the camera's field of view.
    """
    A = max(im_width, im_height)/2
    focal_length = A/math.tan(0.5*math.pi*cam_fov/180)
    ox = im_width/2  # In pixel
    oy = im_height/2
    x = pnt[0]
    y = pnt[1]
    # Project the centre point of the sphere on image plane
    x_im_plane = -(x-ox)/focal_length*0.1
    y_im_plane = -(y-oy)/focal_length*0.1
    z_im_plane = 1*0.1
    return [x_im_plane, y_im_plane, z_im_plane]


def get_camera_fov_planes(camera_params):
    """Calculate the poses of the four planes (left, right, up, down) in the camera's field of view.
    Args:
        im_width: Width of the image in pixels.
        im_height: Height of the image in pixels.
        cam_fov: Camera field of view in degrees.
    Returns:
        A tuple containing the poses of the left, right, up, and down planes as dual quaternions.
    """

    im_width = camera_params["imWidth"]
    im_height = camera_params["imHeight"]
    cam_fov = camera_params["camFov"]

    tra_left = transform_to_image_plane(
        [0, im_height/2], im_width, im_height, cam_fov)
    tra_left_dq = tra_left[0]*i_+tra_left[1]*j_+tra_left[2]*k_
    th_left = math.atan(abs(tra_left[0]/tra_left[2]))
    rot_left = math.cos(-(pi2-th_left)/2)+math.sin(-(pi2-th_left)/2)*j_
    left_pose = rot_left+E_*0.5*tra_left_dq*rot_left

    tra_right = transform_to_image_plane(
        [im_width, im_height/2], im_width, im_height, cam_fov)
    tra_right_dq = tra_right[0]*i_+tra_right[1]*j_+tra_right[2]*k_
    th_right = math.atan(abs(tra_right[0]/tra_right[2]))
    rot_right = math.cos((pi2-th_right)/2)+math.sin((pi2-th_right)/2)*j_
    right_pose = rot_right+E_*0.5*tra_right_dq*rot_right

    tra_up = transform_to_image_plane(
        [im_width/2, 0], im_width, im_height, cam_fov)
    tra_up_dq = tra_up[0]*i_+tra_up[1]*j_+tra_up[2]*k_
    th_up = math.atan(abs(tra_up[1]/tra_up[2]))
    rot_up = math.cos((pi2-th_up)/2)+math.sin((pi2-th_up)/2)*i_
    up_pose = rot_up+E_*0.5*tra_up_dq*rot_up

    tra_down = transform_to_image_plane(
        [im_width/2, im_height], im_width, im_height, cam_fov)
    tra_down_dq = tra_down[0]*i_+tra_down[1]*j_+tra_down[2]*k_
    th_down = math.atan(abs(tra_down[1]/tra_down[2]))
    rot_down = math.cos(-(pi2-th_down)/2)+math.sin(-(pi2-th_down)/2)*i_
    down_pose = rot_down+E_*0.5*tra_down_dq*rot_down

    return (left_pose, right_pose, up_pose, down_pose)


def calculate_plane_jacobian(robot_kin, joint_vals, plane_pose):
    """Calculate the Jacobian of a plane defined by its pose with respect to the robot's end-effector.
    Args:
        robot_kin: The robot kinematic model.
        joint_vals: The joint values of the robot.
        plane_pose: The pose of the plane as a dual quaternion.
    Returns:
        A tuple containing the pose of the plane with respect to the world frame and the Jacobian of the plane.
    """

    plane_pose_wrt_world = robot_kin.fkm(joint_vals)*plane_pose
    plane_pose_jacob = haminus8(plane_pose)@robot_kin.pose_jacobian(joint_vals)
    plane_jacob = robot_kin.plane_jacobian(
        plane_pose_jacob, plane_pose_wrt_world, k_)

    return (plane_pose_wrt_world, plane_jacob)


def cross_product_2d(v1, v2):
    """Compute the 2D cross product of two vectors.
    v1 and v2 are both 2D vectors represented as tuples (x, y) or numpy arrays."""
    return v1[0] * v2[1] - v1[1] * v2[0]

def softmin(distances,  h=0.03, delta=0.1):
    """
    Compute the softmin of a list of distances.

    Parameters:
        distances (list or numpy array): List of distances [F1, F2, ..., Fm].
        h (float): Smoothing parameter (smaller h makes softmin closer to min).

    Returns:
        float: Softmin value.
    """
    # Avoid numerical instability by subtracting the minimum distance
    min_dist = np.min(distances)
    exp_terms = np.exp(-(distances - min_dist) / h)
    softmin_value = -h * np.log(np.mean(exp_terms)) + min_dist - delta
    return softmin_value


def softmin_gradient(distances, gradients, h=0.03):
    """
    Compute the gradient of the softmin function.

    Parameters:
        distances (list or numpy array): List of distances [F1, F2, ..., Fm].
        gradients (list of numpy arrays): List of gradients [∇F1, ∇F2, ..., ∇Fm].
        h (float): Smoothing parameter (smaller h makes softmin closer to min).

    Returns:
        numpy array: Gradient of the softmin function.
    """
    # Avoid numerical instability by subtracting the minimum distance
    min_dist = np.min(distances)
    exp_terms = np.exp(-(distances - min_dist) / h)
    weights = exp_terms / np.sum(exp_terms)  # Normalize to get weights

    # Compute the weighted average of the gradients
    softmin_grad = np.zeros_like(gradients[0])  # Initialize gradient
    for i in range(len(gradients)):
        softmin_grad += weights[i] * gradients[i]

    return softmin_grad
