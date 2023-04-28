
import Jaco2_erobot
import swift as swift
from Simulation_MQTT import MqttClient
import sys
import asyncio
import numpy as np


current_theta_topics = [
            "P2/AAU/CURRENT/theta1",
            "P2/AAU/CURRENT/theta2",
            "P2/AAU/CURRENT/theta3",
            "P2/AAU/CURRENT/theta4",
            "P2/AAU/CURRENT/theta5",
            "P2/AAU/CURRENT/theta6",
            "P2/AAU/CURRENT/finger1",
            "P2/AAU/CURRENT/finger2",
            "P2/AAU/CURRENT/finger3",
            ]
simulation_topics = [
            "P2/AAU/Simulation/theta1",
            "P2/AAU/Simulation/theta2",
            "P2/AAU/Simulation/theta3",
            "P2/AAU/Simulation/theta4",
            "P2/AAU/Simulation/theta5",
            "P2/AAU/Simulation/theta6"]


print("Building enviroment...")
#Init server
mqtt_client = MqttClient()

#Init simulation enviroment
env = swift.Swift()

#Robot assign for simulation
robot_real = Jaco2_erobot.KinovaJaco2()
robot_simulation = Jaco2_erobot.KinovaJaco2()

# Create an empty swift enviroment
env.launch(realtime=True)

#Theta values are set to their zero position
robot_simulation.q = robot_simulation.qr
robot_real.q = robot_real.qr

#Varible for last value received 
last_value_real = []
last_value_simulation = []

#Add the robot model to Swift
real_robot = env.add(robot_real)
simulation_robot = env.add(robot_simulation, robot_alpha=0.4)

#Trajectory values between trajectory visualized 
traj = 2

#Time step in seconds for update of the the graphical scene
try:
    answer_traj = str(input("Do you want to visualize trajectory (y/n): "))
        
except ValueError:
    print("Input is not y/n.")
    sys.exit()
        
if answer_traj == "y":
    trajectory = True

else:
    trajectory = False


print("Simulation running...")

def stop_handler(signal, frame):
    print("Stopping program...")
    for task in asyncio.all_tasks():
        task.cancel()
    sys.exit(0)


while True:
     # Animate the robot
    env.step()

    simulation_values = mqtt_client.subscribe_and_wait_for_messages(simulation_topics)
    if simulation_values != None:
        finger1_theta = 0 #(simulation_values[6]/100)*np.pi/180
        finger2_theta = 0 #(simulation_values[7]/100)*np.pi/180
        finger3_theta = 0 #(simulation_values[8]/100)*np.pi/180
        if last_value_simulation != simulation_values:
            print(simulation_values)

        robot_simulation.q = [simulation_values[0]*np.pi/180,simulation_values[1]*np.pi/180,simulation_values[2]*np.pi/180,simulation_values[3]*np.pi/180,simulation_values[4]*np.pi/180,simulation_values[5]*np.pi/180,
                finger1_theta,0,finger2_theta,0,finger3_theta,0]
        
        #Set the last value as last_value_simulation
        last_value_simulation = simulation_values

        #Checks if the trajectory should be visualized
        if trajectory == True:
            if traj == 0:
                traj_visual = Jaco2_erobot.KinovaJaco2()
                traj_visual.q = robot_simulation.q
                env.add(traj_visual, robot_alpha=0.2)
                traj = 2
            else:
                traj = traj-1

    current_position_robot = mqtt_client.subscribe_and_wait_for_messages(current_theta_topics)
    if current_position_robot != None:
        finger1_theta = (current_position_robot[6]/100)*np.pi/180
        finger2_theta = (current_position_robot[7]/100)*np.pi/180
        finger3_theta = (current_position_robot[8]/100)*np.pi/180
        if last_value_real != current_position_robot:
            print(current_position_robot)

        robot_real.q = [current_position_robot[0]*np.pi/180,current_position_robot[1]*np.pi/180,current_position_robot[2]*np.pi/180,current_position_robot[3]*np.pi/180,current_position_robot[4]*np.pi/180,current_position_robot[5]*np.pi/180,
                    finger1_theta,0,finger2_theta,0,finger3_theta,0]

        #Set the last value as last_value_real
        last_value_real = current_position_robot


