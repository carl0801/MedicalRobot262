import jaco2
import jaco2.move as move
import jaco2.jaco_ik.daniel_kine as dk
import jaco2.jaco_ik.utility as util
import numpy as np
import jaco2.jaco_ik.np_ik as ik
import jaco2.jaco_ik.trajectory_interpolation as ntrp
from jaco2.MQTT import MqttClient
import time
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
    msg = input("Input: ")

    if msg == "line":
        t_s = time.time()
        data = jaco2.get_general_informations()
        act1 = data.actuator1
        act2 = data.actuator2
        act3 = data.actuator3
        act4 = data.actuator4
        act5 = data.actuator5
        act6 = data.actuator6
        #start = np.array([act1, act2, act3, act4, act5, act6])
        start = np.array([0.25,180*np.pi/180,5,1,1,-1.3]) 
        #print("start: ", start)
        start_p = np.deg2rad(start)

        start_pose = dk.fkine(start)
        #print("start_pose: ", start_pose)

        ori = np.array([0,0,0,180*deg,5*deg,-140*deg])

        goal_m_ori = util.pos2mat(ori, 'ypr')
        goal_m = goal_m_ori + start_pose
        goal_ori_theta, info = ik.ik_simple(goal_m)

        #print("goal_ori_theta: ", goal_ori_theta)
        goal = dk.fkine(goal_ori_theta)

        goal[0,3] += float(input("x: "))
        goal[1,3] += float(input("y: "))
        goal[2,3] += -0.4

        #Moving to start
        move_traj, j_vel_traj = ntrp.move_l(start, goal) # (q, target_pos)
        goal[2,3] += -0.05

        #Linje 1
        move_traj2, j_vel_traj2 = ntrp.move_l(move_traj[-1], goal) # (q, target_pos)

        goal[1,3] += 0.1
        move_traj3, j_vel_traj3 = ntrp.move_l(move_traj2[-1], goal) # (q, target_pos)

        goal[2,3] += 0.05
        move_traj4, j_vel_traj4 = ntrp.move_l(move_traj3[-1], goal) # (q, target_pos)

        goal[0,3] += 0.025
        move_traj5, j_vel_traj5 = ntrp.move_l(move_traj4[-1], goal) # (q, target_pos)

        #Linje 2
        goal[2,3] += -0.05
        move_traj6, j_vel_traj6 = ntrp.move_l(move_traj5[-1], goal) # (q, target_pos)

        goal[1,3] += -0.1
        move_traj7, j_vel_traj7 = ntrp.move_l(move_traj6[-1], goal) # (q, target_pos)

        goal[2,3] += 0.05
        move_traj8, j_vel_traj8 = ntrp.move_l(move_traj7[-1], goal) # (q, target_pos)

        goal[0,3] += 0.025
        move_traj9, j_vel_traj9 = ntrp.move_l(move_traj8[-1], goal) # (q, target_pos)

        #Linje 3
        goal[2,3] += -0.05
        move_traj10, j_vel_traj10 = ntrp.move_l(move_traj9[-1], goal) # (q, target_pos)

        goal[1,3] += 0.1
        move_traj11, j_vel_traj11 = ntrp.move_l(move_traj10[-1], goal) # (q, target_pos)

        goal[2,3] += 0.05
        move_traj12, j_vel_traj12 = ntrp.move_l(move_traj11[-1], goal) # (q, target_pos)

        #Moving home
        move_traj13, j_vel_traj13 = ntrp.move_l(move_traj12[-1], start_pose) # (q, target_pos)

        move_traj = np.concatenate((move_traj, move_traj2, move_traj3, move_traj4, move_traj5, move_traj6, move_traj7, move_traj8, move_traj9, move_traj10, move_traj11, move_traj12, move_traj13))
        j_vel_traj = np.concatenate((j_vel_traj, j_vel_traj2, j_vel_traj3, j_vel_traj4, j_vel_traj5, j_vel_traj6, j_vel_traj7, j_vel_traj8, j_vel_traj9, j_vel_traj10, j_vel_traj11, j_vel_traj12, j_vel_traj13))

        t_e = time.time()
        print("Time: ", t_e-t_s)
        move.send_traj(move_traj, j_vel_traj)

    else:
        print("No valid input")
        continue

