import numpy as np
import kinematics as kin
import sys
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator
from scipy.interpolate import interp1d
import scipy.spatial.transform as transform
import jaco2
import time
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180
rad = 180/np.pi

def plot(points):
    hz = 100
    t = np.linspace(0, len(points)/hz,  len(points))
    # Plot the blended points 
    fig, ax1 = plt.subplots()
    ax1.plot(t, points, label='Joint values')
    # Set labels and legends
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Joint values')
    ax1.legend(loc='upper left')
    plt.show()

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
        return np.array([0.25,2.2,5,1,1,-1.3]) 
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
    print(points_array)
    for i in range(len(points)):
        # Compute the new point using kinematics
        point = kin.ik(points[i], points_array[-1],debug)
        # Add the new point to the array
        points_array = np.vstack((points_array, point))
    return points_array

# Points are in xyz,rpy (intrinsic xyz)
def j(p,start=[0,0,0,0,0,0],velocity=28*deg,acceleration=28*deg,debug=False):
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
    times = np.hstack([times[0], times+0.4, times[-1]+0.8]) # Add 0.4s to start and end for de acceleration to zero
    # Create the PchipInterpolator
    ch = PchipInterpolator(times, points, axis=0)
    hz = 100
    t = np.linspace(0, times[-1],  int(np.ceil(times[len(times)-1]/(1/hz))))
    # Evaluate the interpolator at the new time stamps
    joint_traj = ch(t)
    vel_traj = ch(t, 1)
    #plot(traj,t)
    #plot(vel_traj,t)
    print(f"Seconds to complete trajectory: {len(joint_traj)/hz}")
    mqtt_client.publish_message("P2/AAU/op/time", f"{len(joint_traj)/hz}")
    return joint_traj, vel_traj

# Points are in xyz,rpy (intrinsic xyz)
def c(points,start=[0,0,0,0,0,0],velocity=16*deg,acceleration=16*deg,dist_delta=2,debug=False):
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
    joint_traj,vel_traj = j(points_cart,start,velocity,acceleration,debug)
    return joint_traj, vel_traj

def send_traj(joint_traj,vel_traj):
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
    confirmation = input("Confirm trajectory? (y/n): ")
    if confirmation == "y":
        status("Confirmed trajectory!")
        current_time = time.perf_counter()
        time_step = 1/hz
        trajectory = jaco2.TrajectoryPoint()
        trajectory.init()
        trajectory = jaco2.set_type_velocity(trajectory)
        vel_traj = vel_traj*rad
        print("vel_traj size: ", vel_traj.shape)
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

# Be careful with this function, it sends a point not a trajectory.
# We can not simulate this before running it on the robot!
# Points are in xyz,rpy (intrinsic xyz)
def p(point):
    point = np.array([point])
    points = cart_to_joint(point)
    theta = points[1]
    print(f"thetas: {theta}")
    trajectory = jaco2.TrajectoryPoint()
    trajectory.init()
    trajectory = jaco2.set_type_position(trajectory)
    trajectory.actuator1 = theta[0]
    trajectory.actuator2 = theta[1]
    trajectory.actuator3 = theta[2]
    trajectory.actuator4 = theta[3]
    trajectory.actuator5 = theta[4]
    trajectory.actuator6 = theta[5]
    jaco2.send_basic_trajectory(trajectory) 

def workspace(x,y,tool,z_offset=0,yaw_offset=0,joints=[0,0,0,0,0,0]):
    # Needs tuning
    if tool==0: # pen
        # rpy in extrinsic
        roll = 2.8
        pitch = 0.05
        yaw = -2.45
        rpy = [roll,pitch,yaw]
        # convert back to intrinsic
        rpy = transform.Rotation.from_euler('xyz', rpy).as_euler('XYZ')
        x += 419.5
        y += -116
        z_offset += -2
    elif tool==1: # knife
        # tool transform
        transform_rpy = transform.Rotation.from_euler('xyz', [0,0,25*deg]).as_matrix()
        transform_xyz = np.array([45.79,-13.70,93.22])
        transform_matrix = np.hstack((transform_rpy, transform_xyz.reshape(3, 1)))    
        transform_matrix = np.vstack((transform_matrix, np.array([0,0,0,1])))
        # target transform
        transform_rpy = transform.Rotation.from_euler('xyz', [0,0,yaw_offset]).as_matrix()
        transform_xyz = np.array([x,y,0])
        target_matrix = np.hstack((transform_rpy, transform_xyz.reshape(3, 1)))    
        target_matrix = np.vstack((target_matrix, np.array([0,0,0,1])))
        v = transform_matrix @ np.linalg.inv(target_matrix)
        xyz = v[:3,3]
        x = xyz[0]
        y = xyz[1]
        # convert back to intrinsic
        rpy = transform.Rotation.from_matrix(v[:3,:3]).as_euler('xyz')
        # rpy in extrinsic
        roll = 3.14+rpy[0] 
        pitch = 0.0+rpy[1]
        yaw = 0.44+rpy[2]
        rpy = [roll,pitch,yaw]
        # convert back to intrinsic
        rpy = transform.Rotation.from_euler('xyz', rpy).as_euler('XYZ')
        x += 369.5
        y += -76
        z_offset += 47
    # Also needs tuning
    calib_p1 = np.array([419.5,-116,38.7])
    calib_p2 = np.array([404.9,190.2,47.3])
    calib_p3 = np.array([599.3,51.1,49.4])
    v1 = calib_p2 - calib_p1
    v2 = calib_p3 - calib_p1
    n = np.cross(v1,v2)
    n = n/np.linalg.norm(n)
    d = -np.dot(n,calib_p1)
    z = -(n[0]*x + n[1]*y + d)/n[2]
    z += z_offset
    return np.array([x,y,z,rpy[0],rpy[1],rpy[2]])

def home():
    jaco2.move_home()

if __name__ == "__main__":
    """ mqtt_client = MqttClient()
    goal_topics = [
            "P2/AAU/goal/1",
            "P2/AAU/goal/2",
            "P2/AAU/goal/3",
            "P2/AAU/goal/4",
            "P2/AAU/goal/5",
            "P2/AAU/goal/6"]
    print("Waiting for goal points")
    end_point = mqtt_client.subscribe_and_wait_for_messages(goal_topics)
    end_point = np.array(end_point).reshape(1,6)
    print("Goal point received") """
    #j(end_point, 17*deg, 8*deg)
    home()
    """ while True:
        print(get_curr_theta()*rad)
        time.sleep(0.01)
    #Testing som eendeffector rotations
    transform_rpy = transform.Rotation.from_euler('xyz', [0,0,25*deg]).as_matrix()
    transform_xyz = np.array([-8.34,-37.82,93.22])
    transform_matrix = np.hstack((transform_rpy, transform_xyz.reshape(3, 1)))    
    transform_matrix = np.vstack((transform_matrix, np.array([0,0,0,1])))
    print(transform_matrix) """

