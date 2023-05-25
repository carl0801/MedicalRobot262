import numpy as np
import pickle
import os
import time
from jaco_ik.daniel_kine import generate_fk_matrix, compute_jacobian
import numba

def check_qlim(q, qlim):
    """
    Check if any joint exceeds its limits.

    Args:
        q (ndarray): joint angles to check
        qlim (ndarray): joint limits as [min, max] array of size (2,6).
        dev (module, optional): module to compute with, either np (NumPy) or cp (CuPy). Defaults to np.

    Returns:
        Bool: True if any joint exceeds its limits, False otherwise.
    """
    return np.any((q >= qlim[0, :]) & (q <= qlim[1, :]))

@numba.njit
def hess(J):
    """
    Compute the Hessian matrix of a Jacobian matrix.

    Args:
        J (ndarray): Jacobian matrix of size (6, n).

    Returns:
        ndarray: Hessian matrix of size (n, 6, n).
    """
    n = J.shape[1]
    H = np.zeros((n, 6, n))

    for j in range(n):
        for i in range(j, n):
            H[j, :3, i] = np.cross(J[3:, j], J[:3, i])
            H[j, 3:, i] = np.cross(J[3:, j], J[3:, i])

            if i != j:
                H[i, :3, j] = H[j, :3, i]
    return H

def manipulability(J):
    """
    Compute the manipulability of a Jacobian matrix per the Yoshikawa manipulability index.
        See: Siciliano et. al, "Robotics: Modelling, Planning and Control", 2010, p. 126, equation (3.55).

    Args:
        J (ndarray): Jacobian matrix of size (6, n).

    Returns:
        float: manipulability value.
    """
    return np.sqrt(np.linalg.det(J @ J.T))

def jcb_manip(J):
    """
    Compute the manipulability Jacobian

    Args:
        J (ndarray): Jacobian matrix of size (6, n).

    Returns:
        Jm: The manipulability Jacobian
    """
    
    H = hess(J)
    manip = manipulability(J)
    
    coef_a = np.linalg.inv(J @ J.T)
    Jm = np.zeros((J.shape[1], 1))
    
    for i in range(J.shape[1]):
        coef_b = J @ H[i, :, :].T
        Jm[i,0] = manip * np.vdot(coef_a.T, coef_b.T)
    
    return Jm

def joint_limit_cost(q, qlim, pi=1.0, ps=0.2):
    '''
    Compute the joint limit cost function per Corke et. al., Manipulator Differential Kinematics I, 2022
        https://arxiv.org/pdf/2207.01794.pdf, p. 8, equation (49)
        
        Args:
            q (ndarray): joint angles of size (n, 1)
            qlim (ndarray): joint limits as [min, max] array of size (2, n).
            pi (float): distance from limits where penalty starts (rad)
            ps (float): min distance from joint limits (rad) allowed
        
        Returns:
            float: Scalar cost value
        
        '''
    # convert pi to an array of size (n, 1) for broadcasting    
    pi = pi * np.ones(q.shape[-1])

    qlim_lower = qlim[0]
    qlim_upper = qlim[1]

    jlim_cost = np.where(q - qlim_lower <= pi, -((q - qlim_lower - pi) ** 2) / ((ps - pi) ** 2), 0)
    jlim_cost += np.where(qlim_upper - q <= pi, ((qlim_upper - q - pi) ** 2) / ((ps - pi) ** 2), 0)

    return -jlim_cost.flatten()

def qzero(q, J, qlim, we_jl=1.0, we_m=1.0):
    """
    Compute the null-space motion according to the cost functions
        per Corke et. al., Manipulator Differential Kinematics I, 2022
        
        Args:
            q (ndarray): joint angles of size (n, 1)
            J (ndarray): Jacobian matrix of size (6, n).
            we_jl (float): weight for the joint limit cost function
            we_m (float): weight for the manipulability cost function
            
        Returns:
            ndarray: delta joint angles of size (n, 1)
    """
    d_qzero = np.zeros(q.shape[0])
    
    # Compute the joint limit cost function (joint limit avoidance)
    cost = joint_limit_cost(q, qlim)
    d_qzero = d_qzero + (1/we_m * cost).flatten()
    
    # Compute the manipulability cost (singularity avoidance)
    Jm = jcb_manip(J)
    d_qzero = d_qzero + (1/we_jl * Jm).flatten()
    # Compute the null-space motion (null projection @ desired motion)
    dq = (np.eye(q.shape[0]) - np.linalg.pinv(J) @ J) @ d_qzero
    
    return dq

def x_dot(e, k):
    """
    Compute the desired end-effector velocity per Siciliano et. al.
    
    Args:
        e (ndarray): end-effector pose error
        k (float): gain
        
    Returns:
        ndarray: desired end-effector velocity
    """
    
    return (np.eye(6)*k) @ e

def jacob_analytical(jacobian, pose):
    """
    Convert the geometric jacobian to the analytical jacobian. The analytical jacobian is the geometric jacobian with the rotational part transformed to the base frame.

    Args:
        jacobian (tensor): an Nx6X6 tensor representing the geometric jacobians for all poses in the batch
        pose (tensor): an Nx4x4 tensor representing the end-effector pose for all poses in the batch
        dev (module, optional): module to compute with, either np (NumPy) or cp (CuPy). Defaults to np.
    Returns:
        tensor: an Nx6x6 tensor representing the analytical jacobians for all poses in the batch
    """
    zero = np.zeros((jacobian.shape[0], 3, 3), dtype=np.float64)
    I = np.eye(3, dtype=np.float64)
    Id = np.tile(I, (jacobian.shape[0], 1, 1))
    R = pose[:, :3, :3]
    R = np.transpose(R, (0, 2, 1))
    upper = np.concatenate((Id, zero), axis=1)
    #print(f"upper shape: {upper.shape}")
    lower = np.concatenate((zero, R), axis=1)
    #print(f"lower shape: {lower.shape}")
    T = np.concatenate((upper, lower), axis=2)
    #print(f"T shape: {T.shape}")
    # #
    # R = np.concatenate((R, zero), axis=1)
    # print(f"r shape: {R.shape}")
    # print(f"Id shape: {Id.shape}")
    # R = np.concatenate((R, Id), axis=2)
    # R = np.reshape(R, (jacobian.shape[0], 6, 6))
    return np.matmul(T, jacobian)
    

def mat2pos(matrix, rot_order='rpy', output="rad"):
    """
    Convert a 4x4 tensor input representing a transformation matrix to a tensor of X-Y-Z-R-P-Y.

    Args:
        matrix (cupy.ndarray): A 4x4 tensor representing a transformation matrix.
        output (str): An optional argument specifying whether the rotations should be returned in radians or degrees. Defaults to "rad".
        rot_order (str): The order of rotations for the DH convention. 'ypr' or 'rpy'. Default is 'rpy'. 'old' is the same as 'ypr'.
        dev(module): The module to use for the computation. Either np (NumPy) or cp (CuPy). Defaults to np.
    Returns:
        cupy.ndarray: A tensor of X-Y-Z-R-P-Y.
    """

    if rot_order not in ["rpy", "ypr", "old"]:
        raise ValueError("Invalid rotation order. Expected 'rpy', 'ypr'.")

    rotation_matrix = matrix[:3, :3]
    position = matrix[:3, 3]

    if rot_order == "rpy":
        # Extract Roll-Pitch-Yaw angles using the specified rotation order
        rx = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[2, 2]).reshape(1)
        ry = np.arcsin(rotation_matrix[0, 2]).reshape(1)
        rz = np.arctan2(-rotation_matrix[0, 1], rotation_matrix[0, 0]).reshape(1)
        
    else:
        rz = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]).reshape(1)
        ry = np.arcsin(-rotation_matrix[2, 0]).reshape(1)
        rx = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2]).reshape(1)
        
    if output == "rad":
        pass
    elif output == "deg":
        # Convert to degrees
        rx = rx * 180.0 / np.pi
        ry = ry * 180.0 / np.pi
        rz = rz * 180.0 / np.pi
    else:
        raise ValueError("Invalid output format. Expected 'rad' or 'deg'.")
    
    pose = np.concatenate([position, rx, ry, rz])
    pose = np.asarray(pose, dtype=np.float64)
    return pose

def pos2mat(tensor, rotation_order):
    """
    Position coordinates to position transformation matrix,
    
    :param tensor: ndarray of shape [6]
    :param rotation_order: string indicating the order of rotations. Either 'rpy' or 'ypr'.
    :return: 4x4 ndarray representing homogeneous transformation matrix
    """

    
    x, y, z, rx, ry, rz = tensor[0], tensor[1], tensor[2], tensor[3], tensor[4], tensor[5]
    rot_x = np.array([[1, 0, 0],
                      [0, np.cos(rx), -np.sin(rx)],
                      [0, np.sin(rx), np.cos(rx)]])

    rot_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                      [0, 1, 0],
                      [-np.sin(ry), 0, np.cos(ry)]])

    rot_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                      [np.sin(rz), np.cos(rz), 0],
                      [0, 0, 1]])

    if rotation_order == 'rpy':
        R = np.matmul(np.matmul(rot_x, rot_y), rot_z)

    elif rotation_order == 'ypr':
        R = np.matmul(np.matmul(rot_z, rot_y), rot_x)

    else:
        raise ValueError("Invalid rotation order. Must be 'rpy' or 'ypr'")

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array([x, y, z])
    return T


def b_pos2mat(tensor, rotation_order):
    """
    Batched position coordinates to position transformation matrix.
    
    :param tensor: ndarray array of shape [N, 6]
    :param rotation_order: string indicating the order of rotations. Either 'rpy' or 'ypr'.
    :return: Nx4x4 ndarray array representing homogeneous transformation matrices
    """
    
    N = tensor.shape[0]
    matrices = np.zeros((N, 4, 4), dtype=np.float64)

    for i in range(N):
        x, y, z, rx, ry, rz = tensor[i]
        rot_x = np.array([[1, 0, 0],
                          [0, np.cos(rx), -np.sin(rx)],
                          [0, np.sin(rx), np.cos(rx)]])

        rot_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                          [0, 1, 0],
                          [-np.sin(ry), 0, np.cos(ry)]])

        rot_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                          [np.sin(rz), np.cos(rz), 0],
                          [0, 0, 1]])

        if rotation_order == 'rpy':
            R = np.matmul(np.matmul(rot_x, rot_y), rot_z)
        elif rotation_order == 'ypr':
            R = np.matmul(np.matmul(rot_z, rot_y), rot_x)
        else:
            raise ValueError("Invalid rotation order. Must be 'rpy' or 'ypr'")

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = np.array([x, y, z])
        matrices[i] = np.asarray(T)

    return matrices

def get_vel_traj(joint_traj):
    vel_traj = np.zeros_like(joint_traj)
    for i in range(joint_traj.shape[0]-1):
        vel_traj[i+1] = joint_traj[i+1] - joint_traj[i]
    return vel_traj
