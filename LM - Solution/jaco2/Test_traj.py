import jaco2
import move as move
import jaco_ik.daniel_kine as dk
import numpy as np
import jaco_ik.trajectory_interpolation as ntrp
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180

dk.setup_module()
jaco2.init_robot()

goal_theta_topics = [
            "P2/AAU/goalAngles/1",
            "P2/AAU/goalAngles/1",
            "P2/AAU/goalAngles/1",
            "P2/AAU/goalAngles/1",
            "P2/AAU/goalAngles/1",
            "P2/AAU/goalAngles/1"]


while True:
    data = jaco2.get_general_informations()
    act1 = data.actuator1
    act2 = data.actuator2
    act3 = data.actuator3
    act4 = data.actuator4
    act5 = data.actuator5
    act6 = data.actuator6
    start = np.deg2rad(np.array([act1, act2, act3, act4, act5, act6])) 

    start_pose = dk.fkine(start)

    goal = start_pose.copy()
    goal[0,3] += float(input("x: "))
    goal[1,3] += float(input("y: ")) 
    goal[2,3] += float(input("z: "))

    move_traj, j_vel_traj = ntrp.move_l(start, goal) # (q, target_pos)
    move.send_traj(move_traj, j_vel_traj)

    





