import numpy as np
import kinematics as kin
#import jaco_inverse.np_ik as kin2
import sys
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator
from scipy.interpolate import interp1d
import scipy.spatial.transform as transform
from Jaco2_erobot import KinovaJaco2
robot = KinovaJaco2()
import jaco2
jaco2.init_robot()
import time
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180
rad = 180/np.pi

# global vars
global_traj_joint = np.empty((0,6))
global_traj_vel = np.empty((0,6))
cur_tool = "none"

def plot():
    points = global_traj_joint
    # plot joint values
    hz = 100
    t = np.linspace(0, len(points)/hz,  len(points))
    # Plot the blended points 
    fig, ax1 = plt.subplots()
    joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
    # Iterate over each joint index and plot its values with the corresponding label
    for i in range(global_traj_joint.shape[1]):
        joint_values = global_traj_joint[:, i]
        joint_label = joint_labels[i]
        ax1.plot(t, joint_values, label=joint_label)
    # Set labels and legends
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Joint values (radians)')
    ax1.legend(loc='lower left')
    plt.show()
    # Simulate locally
    robot.plot(np.pad(points[::2],(0,6)),dt=0.01) # change [::1] to [::2] for 2x speed

def status(msg):
    mqtt_client.publish_message("P2/AAU/info", msg)
    print(f"Status: {msg}")

def get_curr_theta():
    current_theta_topics = [
            "P2/AAU/current/theta1",
            "P2/AAU/current/theta2",
            "P2/AAU/current/theta3",
            "P2/AAU/current/theta4",
            "P2/AAU/current/theta5",
            "P2/AAU/current/theta6"]
    curr_theta, recived_data = mqtt_client.subscribe_with_timeout(current_theta_topics, timeout=3)
    if not recived_data:
        print("No data recieved! Robot might not be connected!")
        return np.array([0.25,3.14,5,1,1,-1.3]) 
    #curr_theta = mqtt_client.subscribe_and_wait_for_messages(current_theta_topics)
    curr_theta = np.array(curr_theta)*deg
    return curr_theta

def cart_to_joint(points,curr=np.array([0,0,0,0,0,0]),debug=False):
    if curr.all()==0:
        # Get current position
        start_point = get_curr_theta()
    else: 
        start_point = curr
    # Add the start point to the array
    points_array = np.array([start_point])
    for i in range(len(points)):
        # Compute the new point using kinematics
        point = kin.ik(points[i], points_array[-1],debug)
        # Add the new point to the array
        points_array = np.vstack((points_array, point))
    return points_array

# Points are in xyz,rpy (intrinsic xyz)
def generate_j(p,start=[0,0,0,0,0,0],velocity=28*deg,acceleration=28*deg,debug=False):
    start=np.array(start)
    points = np.array(p)
    points = cart_to_joint(points,start,debug)
    # Find adjacent duplicates
    diffs = np.diff(points, axis=0)
    adjacent_duplicates = np.where(np.all(diffs == 0, axis=1))[0]
    # Remove adjacent duplicates
    points = np.delete(points, adjacent_duplicates + 1, axis=0)
    # Generate interpolation
    max_acceleration = acceleration
    max_velocity = velocity
    # Calculate the maximum achievable velocity and acceleration for each segment
    deltas = np.diff(points, axis=0) # vector between each point
    distances = np.linalg.norm(deltas, axis=1) # distance between each point
    max_velocities = np.minimum(np.sqrt(2 * max_acceleration * distances), max_velocity) # velocity at each point
    max_accelerations = np.minimum(max_velocities**2 / (2 * distances), max_acceleration) # acceleration at each point
    # Calculate the time stamps
    times = [0]
    for i, (delta, max_velocity, max_acceleration) in enumerate(zip(deltas, max_velocities, max_accelerations)):
        distance = distances[i]
        time_to_max_vel = max_velocity / max_acceleration
        time_to_stop = max_velocity / max_acceleration
        time_to_travel = distance / max_velocity
        if time_to_travel < time_to_max_vel + time_to_stop:
            # The segment can be traveled at maximum velocity
            time_stamp = time_to_travel
        else:
            # The segment requires acceleration and deceleration
            time_stamp = (max_velocity / max_acceleration) * 2 + (distance - max_velocity**2 / (2 * max_acceleration)) / max_velocity * 2
        times.append(times[-1] + time_stamp)
    times = np.array(times)
    points = np.vstack([points[0, :], points, points[-1, :]])
    times = np.hstack([times[0], times+0.35, times[-1]+0.7]) # Add 0.35s to start and end for de acceleration to zero
    # Create the PchipInterpolator
    ch = PchipInterpolator(times, points, axis=0)
    hz = 100
    t = np.linspace(0, times[-1],  int(np.ceil(times[len(times)-1]/(1/hz))))
    # Evaluate the interpolator at the new time stamps
    joint_traj = ch(t)
    vel_traj = ch(t, 1)
    #plot(traj,t)
    #plot(vel_traj,t)
    print(f"Calculated trajectory! {len(joint_traj)/hz}s to complete")
    return joint_traj, vel_traj

# Points are in xyz,rpy (intrinsic xyz)
def generate_l(points,start=[0,0,0,0,0,0],velocity=16*deg,acceleration=16*deg,dist_delta=2,debug=False):
    start=np.array(start)
    # Convert xyz,rpy to matrix
    tensor = []
    for i in range(len(points)):
        m = np.eye(4)
        m[:3,3] = points[i,:3]
        m[:3,:3] = transform.Rotation.from_euler('XYZ', points[i,3:]).as_matrix()
        tensor.append(m)
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
    # Transform back to xyz,rpy
    points_cart = []
    for i in range(len(traj)):
        rpy = transform.Rotation.from_rotvec(traj[i,3:]).as_euler('XYZ')
        points_cart.append(np.concatenate([traj[i,:3], rpy]))
    points_cart = np.array(points_cart[1:])
    joint_traj,vel_traj = generate_j(points_cart,start,velocity,acceleration,debug)
    return joint_traj, vel_traj

def send_traj():
    global global_traj_joint
    global global_traj_vel
    joint_traj = global_traj_joint
    vel_traj = global_traj_vel
    hz = 100
    # Simulate trajectory
    status("Simulating...")
    i = 0
    current_time = time.perf_counter()
    time_step = (1/hz)*20
    while i < len(joint_traj):
        mqtt_client.publish_message("P2/AAU/simulation/theta1", joint_traj[i][0]*rad)
        mqtt_client.publish_message("P2/AAU/simulation/theta2", joint_traj[i][1]*rad)
        mqtt_client.publish_message("P2/AAU/simulation/theta3", joint_traj[i][2]*rad)
        mqtt_client.publish_message("P2/AAU/simulation/theta4", joint_traj[i][3]*rad)
        mqtt_client.publish_message("P2/AAU/simulation/theta5", joint_traj[i][4]*rad)
        mqtt_client.publish_message("P2/AAU/simulation/theta6", joint_traj[i][5]*rad)
        # busy loop to wait until the time step has elapsed
        while time.perf_counter() - current_time < time_step:
            pass
        # update the current time
        current_time += time_step
        i += 40
    # Wait for confirmation
    status("Waiting for confirmation...")
    confirmation = mqtt_client.subscribe_and_wait_for_messages(["P2/AAU/op"])
    if confirmation[0] == 1:
        status("Confirmed trajectory!")
        current_time = time.perf_counter()
        time_step = 1/hz
        trajectory = jaco2.TrajectoryPoint()
        trajectory.init()
        trajectory = jaco2.set_type_velocity(trajectory)
        vel_traj = vel_traj*rad
        for i in range(len(vel_traj)):
            vel = vel_traj[i]
            trajectory.actuator1 = vel[0]
            trajectory.actuator2 = vel[1]
            trajectory.actuator3 = vel[2]
            trajectory.actuator4 = vel[3]
            trajectory.actuator5 = vel[4]
            trajectory.actuator6 = vel[5]
            jaco2.send_basic_trajectory(trajectory) 
            # busy loop to wait until the time step has elapsed
            while time.perf_counter() - current_time < time_step:
                pass
            # update the current time
            current_time += time_step
        status("Robot done with trajectory!")
    else:
        status("Declined trajectory!")
        sys.exit("GUI declined trajectory!")
    # reset global traj
    global_traj_joint = np.empty((0,6))
    global_traj_vel = np.empty((0,6))

def mat_to_xyz_rpy_intrin(matrix):
    x = matrix[0,3]
    y = matrix[1,3]
    z = matrix[2,3]
    # convert to intrinsic
    rpy = transform.Rotation.from_matrix(matrix[:3,:3]).as_euler('XYZ')
    return x,y,z,rpy

def workspace(x,y,z=0,x_rot=0,y_rot=0,z_rot=0):
    # workspace transform w-b
    workspace_matrix = np.eye(4)
    workspace_matrix[:3,3] = np.array([-344.5,81,0])
    # target transform w-t
    # note xyz is extrinsic, and XYZ is intrinsic
    target_matrix = np.eye(4)
    target_matrix[:3,:3] = transform.Rotation.from_euler('xyz', [x_rot,y_rot,z_rot]).as_matrix()
    target_matrix[:3,3] = np.array([x,y,z])
    if cur_tool=="pen":
        # tool transform e-t
        tool_matrix = np.eye(4)
        tool_matrix[:3,:3] = transform.Rotation.from_euler('xyz', [180*deg,5*deg,-140*deg]).as_matrix()
        tool_matrix[:3,3] = np.array([40,40,28.5])
        # Base to end effector matrix
        base_matrix = np.linalg.inv(workspace_matrix) @ target_matrix @ np.linalg.inv(tool_matrix)
        x,y,z,rpy = mat_to_xyz_rpy_intrin(base_matrix)
    elif cur_tool=="knife":
        # tool transform e-t
        tool_matrix = np.eye(4)
        tool_matrix[:3,:3] = transform.Rotation.from_euler('xyz', [180*deg,0,25*deg]).as_matrix()
        tool_matrix[:3,3] = np.array([45.79,-13.70,18]) # old values [-7.46,-38.51,(93.22)]
        # Base to end effector matrix
        base_matrix = np.linalg.inv(workspace_matrix) @ target_matrix @ np.linalg.inv(tool_matrix)
        x,y,z,rpy = mat_to_xyz_rpy_intrin(base_matrix)
    elif cur_tool=="none":
        # Does not transform to any tool
        # Base to end effector matrix
        base_matrix = target_matrix
        x,y,z,rpy = mat_to_xyz_rpy_intrin(base_matrix)
    # Calibrate Z
    calib_p1 = np.array([419.5,-116,38.7])
    calib_p2 = np.array([404.9,190.2,47.3])
    calib_p3 = np.array([599.3,51.1,49.4])
    v1 = calib_p2 - calib_p1
    v2 = calib_p3 - calib_p1
    n = np.cross(v1,v2)
    n = n/np.linalg.norm(n)
    d = -np.dot(n,calib_p1)
    z += -(n[0]*x + n[1]*y + d)/n[2]
    # return result
    """ print(f"target: {target_matrix}")
    print(f"tool: {tool_matrix}")
    print(f"work: {workspace_matrix}")
    print(f"base: {base_matrix}") """
    result = np.array([x,y,z,rpy[0],rpy[1],rpy[2]])
    return result

def change_tool(tool):
    # Changes global tool var
    global cur_tool
    if tool=="pen":
        cur_tool = tool
    elif tool=="knife":
        cur_tool = tool
    elif tool=="none":
        cur_tool = tool
    else:
        print("Invalid tool name!")
    status(f"Changed tool to ({cur_tool})")

def j(matrix,speed=32*deg,start=False):
    # make a traj in joint space and append to global traj
    # matrix is x,y,z,x_rot,y_rot,z_rot
    global global_traj_joint
    global global_traj_vel
    # transform matrix into correct shape
    # Given array
    zero_matrix = np.zeros((len(matrix), 6))
    # Copy the values from the original array to the new array
    for i, row in enumerate(matrix):
        zero_matrix[i, :len(row)] = row
    matrix = zero_matrix
    # make a target matrix defined by workspace and tool
    target = np.empty((0,6))
    for m in matrix:
        target = np.vstack((target,workspace(m[0],m[1],m[2],m[3],m[4],m[5])))
    # make a traj in joint space
    if start:
        start_pos = [0,0,0,0,0,0]
    else: 
        start_pos = global_traj_joint[-1]
    traj_joint, traj_vel = generate_j(target,start_pos,speed,speed)
    # append to global traj
    global_traj_joint = np.vstack((global_traj_joint,traj_joint))
    global_traj_vel = np.vstack((global_traj_vel,traj_vel))

def l(matrix,speed=32*deg,start=False):
    # make a linear traj in cartesian space and append to global traj
    # matrix is x,y,z,x_rot,y_rot,z_rot
    global global_traj_joint
    global global_traj_vel
    # transform matrix into correct shape
    # Given array
    zero_matrix = np.zeros((len(matrix), 6))
    # Copy the values from the original array to the new array
    for i, row in enumerate(matrix):
        zero_matrix[i, :len(row)] = row
    matrix = zero_matrix
    # make a target matrix defined by workspace and tool
    target = np.empty((0,6))
    for m in matrix:
        target = np.vstack((target,workspace(*m)))
    # make a traj in joint space
    if start:
        start_pos = [0,0,0,0,0,0]
    else: 
        start_pos = global_traj_joint[-1]
    traj_joint, traj_vel = generate_l(target,start_pos,speed,speed)
    # append to global traj
    global_traj_joint = np.vstack((global_traj_joint,traj_joint))
    global_traj_vel = np.vstack((global_traj_vel,traj_vel))

def home():
    jaco2.move_home()

if __name__ == "__main__":
    change_tool("pen")
    j([[0,0],[10,10]],start=True)
    j([[0,0]])
    l([[100,100]])
    #plot()
    #send_traj()