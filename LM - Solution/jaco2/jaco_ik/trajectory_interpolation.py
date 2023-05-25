import numpy as np
from scipy.interpolate import CubicSpline
import roboticstoolbox as rtb
from spatialmath import SE3
import jaco_ik.utility as util
import jaco_ik.np_ik as ik
import time
import jaco_ik.interpolation as interpolation
import jaco_ik.daniel_kine as dk

def tensor_to_se3(tensor):
    """
    Convert tensor of X-Y-Z-Roll-Pitch-Yaw to an SE3 object.

    Args:
        tensor (numpy.ndarray): A 1-D array of length 6 containing X, Y, Z, roll, pitch, and yaw values.

    Returns:
        SE3: An SE3 object representing the transformation matrix.
    """

    bottom_row = np.array([0, 0, 0, 1])
    rot_mat = util.pos2mat('xyz', tensor[3:6])
    tf = np.concatenate((rot_mat, tensor[:3].reshape(3, 1)), axis=1)
    tf = np.concatenate((tf, bottom_row.reshape(1, 4)), axis=0)
    
    return SE3(tf)



def velocities(intermediate_poses, # list of poses in trajectory
               dt=0.01, # delta time between poses
               accel=30, # Accceleration in mm/s^2
               max_speed=36 # Max speed in mm/s
               ):
    
    velocities = []
    
    num_poses = len(intermediate_poses) + 2 # Add 2 for the endpoints
    accel_per_step = accel * dt
    accel_time_in_steps = max_speed / accel_per_step


    if accel_time_in_steps*2 > num_poses:
        accel_time_in_steps = (num_poses-1)/2 # -1 to have a step with constant vel before decel
        
    for i in range(len(intermediate_poses)):
        vel = None
    
    return None

def move_j(q, target_pos, debug=False):
    """
    Generates a trajectory between two points in joint space. No guarantees for cartesian space safety.
    
    Args:
        q (np.ndarray): Current joint angles
        target_pos (np.ndarray): 4x4 transformation matrix of target position in cartesian space
        debug (bool, optional): If True, saves the joint space trajectory to a file. Defaults to False.
        
    Returns:
        np.ndarray: Joint space trajectory
    """

    joint_angles, _ = ik.ik_simple(target_pos, q)
    
    joint_trajectory_endpoints = np.stack([q, joint_angles])
    # Scale joint angles so they are within 0 to 360 degrees
    joint_trajectory_endpoints = joint_trajectory_endpoints % (2*np.pi)

    # Scale joint angles so they are continuous
    for j in range(6):
        for i in range(len(joint_trajectory_endpoints)-1):
            diff = joint_trajectory_endpoints[i, j] - joint_trajectory_endpoints[i+1, j]
            if abs(diff) > np.pi:
                joint_trajectory_endpoints[i+1:, j] = joint_trajectory_endpoints[i+1:, j] + np.sign(diff)*2*np.pi


    #num_points = int(np.ceil(np.linalg.norm(joint_trajectory_endpoints[0] - joint_trajectory_endpoints[1])))
    #j_traj_via = rtb.tools.trajectory.jtraj(joint_trajectory_endpoints[0], joint_trajectory_endpoints[1], num_points, qd0=None, qd1=None)
    #j_traj = rtb.tools.trajectory.mstraj(j_traj_via.q, dt=0.01, tacc=1, qdmax=15)
    #joint_trajectory = j_traj.q
    #j_vel_traj = util.get_vel_traj(joint_trajectory)

    if debug:
        np.save("move_traj_j_pos.npy", joint_trajectory)
        np.save("move_traj_j_vel.npy", j_vel_traj)

    return joint_trajectory, j_vel_traj


def move_l(q, target_pos, debug=False):
    """
    Generates two subsequent straight lines, from current to start, and from start to end.
    If line_start is None, the current position is used as the start position (one move).
    """
    
    """
    
    Structure:
        - Compute current position from q
        - Interpolate cartesian traj. from current to target
        - Do IK for each pose in the cartesian trajectory -> get joint space viapoints
        - Interpolate with joint via points to get joint space trajectory
    
    """

    start_pos = dk.fkine(q)
    start_pos_mm = start_pos.copy()
    target_pos_mm = target_pos.copy()
    start_pos_mm[:3, 3] = start_pos[:3, 3]*1000
    target_pos_mm[:3, 3] = target_pos[:3, 3]*1000
    # line_start and line_end are 6D vectors of the form [x, y, z, roll, pitch, yaw] (in intrinsic rotations)

    #print(f"diff start and target pose: \n{start_pos_mm - target_pos_mm}")
    # Interpolate cartesian trajectory between start and target
    start_se3 = SE3(start_pos_mm)
    print(target_pos_mm)
    target_se3 = SE3(target_pos_mm)
    
    #cart_trajectory_desired = interpolation.ctraj(np.stack([start_pos, target_pos]))
    start_target_xyz_dist = int(np.ceil(np.linalg.norm(start_pos_mm[:3, 3] - target_pos_mm[:3, 3])))
    print(f"num_points: {start_target_xyz_dist}")
    cart_trajectory_se3 = rtb.tools.trajectory.ctraj(start_se3, target_se3, t=start_target_xyz_dist)
    print(f"cart_trajectory_se3 length: {len(cart_trajectory_se3)}")
    cart_trajectory_desired = np.array(cart_trajectory_se3.A)
    cart_trajectory_desired[:, :3, 3] = cart_trajectory_desired[:, :3, 3] / 1000 # convert back to meters
    # get trajectory as 4x4 matrices describing each pose in the trajectory
    print(f"cart_trajectory_desired shape: {cart_trajectory_desired.shape}")
    joint_trajectory_via = [q]
    cart_trajectory_via = [start_pos]

    start_time = time.time()
    # do IK for each pose in the trajectory from start to target
    for i in range(len(cart_trajectory_desired)-1):
        joint_angles, _ = ik.ik_simple(cart_trajectory_desired[i+1], joint_trajectory_via[i])
        joint_trajectory_via.append(joint_angles)
        cart_trajectory_via.append(dk.fkine(joint_angles))

    joint_trajectory_via = np.stack(joint_trajectory_via)
    cart_trajectory_via = np.stack(cart_trajectory_via)

    if debug:
        np.save("joint_trajectory_via.npy", joint_trajectory_via)
        np.save("cart_trajectory_via.npy", cart_trajectory_via)
        print()
        print(f"------- {time.time()-start_time} seconds -------")
        print()

    # Scale joint angles so they are within 0 to 360 degrees
    joint_trajectory_via = joint_trajectory_via % (2*np.pi)

    # Scale joint angles so they are continuous
    for j in range(6):
        for i in range(len(joint_trajectory_via)-1):
            diff = joint_trajectory_via[i, j] - joint_trajectory_via[i+1, j]
            if abs(diff) > np.pi:
                joint_trajectory_via[i+1:, j] = joint_trajectory_via[i+1:, j] + np.sign(diff)*2*np.pi

    #joint_trajectory_via = np.pad(joint_trajectory_via, ((0, 0), (0, 6)), "constant", constant_values=0)
    joint_space_trajectory, j_vel_traj = interpolation.mstraj(joint_trajectory_via)

    np.save("move_traj_j_pos.npy", joint_space_trajectory)
    np.save("move_traj_j_vel.npy", j_vel_traj)

    return joint_space_trajectory, j_vel_traj

def gen_line_trajectory(q, line_start=None, line_end=None):
    """
    Generates two subsequent straight lines, from current to start, and from start to end.
    If line_start is None, the current position is used as the start position (one move).
    """
    
    """
    
    Structure:
        - Comput current position from q
        - Interpolate cartesian traj. from current to start
        - Interpolate cartesian traj. from start to end
        - Append the two trajectories to get the full cartesian trajectory
        - Do IK for each pose in the full cartesian trajectory
        - Interpolate each q pose to get full joint space trajectory
    
    """

    
    
    
    if line_start is None:
        line_start = dk.fkine(q)
    # line_start and line_end are 6D vectors of the form [x, y, z, roll, pitch, yaw] (in intrinsic rotations)
    start_pos = line_start
    target_pos = line_end



    # Interpolate cartesian trajectory between start and target
    traj = interpolation.ctraj(np.stack([start_pos, target_pos]))
    # get trajectory as 4x4 matrices describing each pose in the trajectory
    trajectory = np.array(traj)
    
    # do first IK to find joint angles for start_pos
    j0, c0 = ik.ik_simple(q, start_pos)
    joint_trajectory = [j0]
    cart_trajectory = [c0]

    start_time = time.time()
    # do IK for each pose in the trajectory from start to target
    #print(f"len(trajectory): {len(trajectory)}")
    for i in range(len(trajectory)-1):
        joint_angles, cart_pos = ik.ik_simple(joint_trajectory[i], trajectory[i+1])
        joint_trajectory.append(joint_angles)
        cart_trajectory.append(cart_pos)
    
    #print()
    #print(f"------- {time.time()-start_time} seconds -------")
    #print()

    joint_trajectory = np.stack(joint_trajectory)
    cart_trajectory = np.stack(cart_trajectory)


    # Scale joint angles so they are within 0 to 360 degrees
    joint_trajectory = joint_trajectory % (2*np.pi)

    # Scale joint angles so they are continuous
    for j in range(6):
        for i in range(len(joint_trajectory)-1):
            diff = joint_trajectory[i, j] - joint_trajectory[i+1, j]
            if abs(diff) > np.pi:
                joint_trajectory[i+1:, j] = joint_trajectory[i+1:, j] + np.sign(diff)*2*np.pi

    move_to_start = np.stack([q, joint_trajectory[0]])
    move_to_start_pad = np.pad(move_to_start, ((0, 0), (0, 6)), "constant", constant_values=0)

    move_to_start_trajectory, move_start_vel = interpolation.mstraj(move_to_start_pad)

    np.save("move_to_start_j_trajectory.npy", move_to_start_trajectory)
    joint_trajectory = np.pad(joint_trajectory, ((0, 0), (0, 6)), "constant", constant_values=0)

    joint_space_trajectory, j_vel_traj = interpolation.mstraj(joint_trajectory)

    np.save("start_to_target_j_trajectory.npy", joint_space_trajectory)
    full_trajectory = np.concatenate((move_to_start_trajectory, joint_space_trajectory), axis=0)

    full_vel_traj = np.concatenate((move_start_vel, j_vel_traj), axis=0)

    np.save("full_joint_space_trajectory.npy", full_trajectory)
    np.save("full_joint_space_velocity_trajectory.npy", full_vel_traj)

    return full_trajectory, full_vel_traj