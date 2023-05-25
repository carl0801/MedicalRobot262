import jaco2
import jaco2.move as move
import jaco2.jaco_ik.daniel_kine as dk
import jaco2.jaco_ik.utility as util
import numpy as np
import jaco2.jaco_ik.trajectory_interpolation as ntrp
import jaco2.jaco_ik.np_ik as ik
from jaco2.MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180

jaco2.init_robot()

goal_theta_topics = [
            "P2/AAU/goal/1",
            "P2/AAU/goal/2",
            "P2/AAU/goal/3",
            "P2/AAU/goal/4",
            "P2/AAU/goal/5",
            "P2/AAU/goal/6"]

jaco2.move_home()
while True:
    data = jaco2.get_general_informations()
    act1 = data.actuator1
    act2 = data.actuator2
    act3 = data.actuator3
    act4 = data.actuator4
    act5 = data.actuator5
    act6 = data.actuator6
    start = np.array([act1, act2, act3, act4, act5, act6])
    print("start: ", start)
    start = np.deg2rad(start)

    start_pose = dk.fkine(start)
    start_values = util.mat2pos(start_pose, 'ypr')

    #goal_theta = np.array(mqtt_client.subscribe_and_wait_for_messages(goal_theta_topics))
    #goal_theta = np.array([0.5,0.3,0.4,0,0,0])

    goal = start_pose.copy()
    goal[0,3] += float(input("x: "))
    goal[1,3] += float(input("y: ")) 
    goal[2,3] += float(input("z: "))

    #goal_theta[0] = goal_theta[0]/1000
    #goal_theta[1] = goal_theta[1]/1000
    #goal_theta[2] = goal_theta[2]/1000
    #goal_theta[3] = start_values[3]
    #goal_theta[4] = start_values[4]
    #goal_theta[5] = start_values[5]

    #print("goal_theta: ", goal_theta)
    #angles_deg = np.rad2deg(goal_theta[3:])
    #print("angles_deg: ", angles_deg)   
    #goal = util.pos2mat(goal_theta, 'ypr')

    move_traj, j_vel_traj = ntrp.move_l(start, goal) # (q, target_pos)
    #print("move_traj: ", move_traj)
    #print("j_vel_traj: ", j_vel_traj)
    print("Calculated traj: degrees", np.rad2deg(move_traj[-1]))

    move.send_traj(move_traj, j_vel_traj)
    data = jaco2.get_general_informations()
    act1 = data.actuator1
    act2 = data.actuator2
    act3 = data.actuator3
    act4 = data.actuator4
    act5 = data.actuator5
    act6 = data.actuator6
    end_pos = np.array([act1, act2, act3, act4, act5, act6])
    print("end_pos: ", end_pos)
    print("diffend: ", end_pos - np.rad2deg(move_traj[-1]))






