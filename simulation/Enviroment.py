
import Jaco2_erobot
import swift as swift
from MQTT import MqttClient
import sys

print("Building enviroment...")
#Init server
server = MqttClient()

#Init simulation enviroment
env = swift.Swift()

#Robot assign for simulation
robot = Jaco2_erobot.KinovaJaco2()

# Create an empty swift enviroment
env.launch(realtime=True)

#Theta values are set to their zero position
robot.q = robot.qr

#Add the robot model to Swift
main_robot = env.add(robot)

#Trajectory values between trajectory visualized 
traj = 1

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

while True:
    # Animate the robot
    env.step()

    current_position = server.subscribe_and_wait_for_messages()
    print(current_position)
    robot.q = [current_position[0],current_position[1],current_position[2],current_position[3],current_position[4],current_position[5],
               0,0,0,0,0,0]

    #Checks if the trajectory should be visualized
    if trajectory == True:
        if traj == 0:
            traj_visual = Jaco2_erobot.KinovaJaco2()
            traj_visual.q = robot.q
            env.add(traj_visual, robot_alpha=0.2)
            traj = 1
        else:
            traj = traj-1


