import numpy as np
from scipy.interpolate import PchipInterpolator, interp1d
import scipy.spatial.transform as transform

deg = np.pi/180 # deg to rad

# interpolate transformation matricies
def ctraj(tensor,dist_delta=1):
    # tensor: list of transformation matricies
    # dist_delta: distance between each transformation matrix
    # Return: list of transformation matricies
    interpolation_points = []
    # Transform matrix to x,y,z, rot_vec
    for i in range(len(tensor)):
        m = tensor[i]
        v = transform.Rotation.from_matrix(m[:3,:3]).as_rotvec()
        interpolation_points.append(np.concatenate([m[:3,3], v]))
    interpolation_points = np.array(interpolation_points) 
    # Generate linear interpolation of points
    l_traj = interp1d(np.linspace(0, 1, len(interpolation_points)), interpolation_points, axis=0)
    deltas = np.diff(interpolation_points[:3], axis=0) # vector between each point
    distances = np.linalg.norm(deltas, axis=1) # distance between each point
    interpolation_points = int(np.ceil(np.sum(distances)/dist_delta)) # number of interpolation points
    traj = l_traj(np.linspace(0, 1, interpolation_points)) # generate array from scipy interpolation
    # Transform xyz,rot_vec back to matrix
    tensor = []
    for i in range(len(traj)):
        m = np.eye(4)
        m[:3,3] = traj[i,:3]
        m[:3,:3] = transform.Rotation.from_rotvec(traj[i,3:]).as_matrix()
        tensor.append(m)
    return tensor

# interpolate joint angles
def mstraj(matrix,max_velocity=15,max_acceleration=15):
    # matrix: list of joint angles
    # max_velocity: maximum velocity of joints
    # max_acceleration: maximum acceleration of joints
    # Return: list of joint angles and list of joint velocities
    interpolation_points = []

    # find adjacent duplicates
    diffs = np.diff(matrix, axis=0)
    adjacent_duplicates = np.where(np.all(diffs == 0, axis=1))[0]
    # Remove adjacent duplicates
    matrix = np.delete(matrix, adjacent_duplicates + 1, axis=0)

    # find ideal spacing of points
    # Calculate the maximum achievable velocity and acceleration for each segment
    deltas = np.diff(matrix, axis=0) # vector between each point
    distances = np.linalg.norm(deltas, axis=1) # distance between each point
    # max achivable velocity at each point
    # Using the kinematic equation v²=v_0²+2as where a start velocity of zero is assumed
    max_velocities = np.minimum(np.sqrt(2 * max_acceleration * distances), max_velocity) 
    # max achvievable acceleration at each point
    # using the formula same formula but rearranged to a = (v²-v_0²)/2s
    max_accelerations = np.minimum(max_velocities**2 / (2 * distances), max_acceleration) 
    # Calculate the time stamps
    times = [0]
    for i, (distance, max_velocity, max_acceleration) in enumerate(zip(distances, max_velocities, max_accelerations)):
        # m/s / m/s² = s
        time_to_max_vel = max_velocity / max_acceleration
        # m / m/s = s
        time_to_travel = distance / max_velocity
        # if time_to_travel is less than accel adn deccel time 
        # we assume a good time_stamp will be the time to travel at max velocity
        if time_to_travel < time_to_max_vel*2:
            # The segment can be traveled at maximum velocity
            time_stamp = time_to_travel
        else:
            # The segment requires acceleration and deceleration
            # The time to accelerate to max velocity is the same as the time to decelerate to zero
            # We calulate the time traveld at max accelaration and add it to 
            time_stamp = time_to_max_vel * 2 + (distance - max_velocity**2 / (2 * max_acceleration)) / max_velocity * 2
        times.append(times[-1] + time_stamp)
    times = np.array(times)
    # Add 0.5s to start and 0.5s to end for de acceleration to zero
    matrix = np.vstack([matrix[0, :], matrix, matrix[-1, :]])
    times = np.hstack([times[0], times+0.5, times[-1]+1.0])
    # Create the PchipInterpolator
    ch = PchipInterpolator(times, matrix, axis=0)
    hz = 100
    t = np.linspace(0, times[-1],  int(np.ceil(times[len(times)-1]/(1/hz))))
    # Evaluate the interpolator at the new time stamps
    joint_traj = ch(t)
    vel_traj = ch(t, 1)
    #plot(traj,t)
    #plot(vel_traj,t)
    print(f"Seconds to complete trajectory: {len(joint_traj)/hz}")
    return joint_traj,vel_traj


