import move
import numpy as np
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180

move.status("Starting operation_point_move.py")

# Get points
move.status("Waiting for coords...")
operation_setting_topics = [
    "P2/AAU/goal/1", # x
    "P2/AAU/goal/2", # y
    "P2/AAU/goal/3", # z
    "P2/AAU/goal/4", # x_r
    "P2/AAU/goal/5", # y_r
    "P2/AAU/goal/6"] # z_r
settings = mqtt_client.subscribe_and_wait_for_messages(operation_setting_topics)
move.status("Coords recived!")
#settings = [100,100,100,0,0,0]
# transform deg to rad
settings = np.array(settings)
settings[-3:] *= deg
print(f"settings: {settings}")
# Define trajectories
move.change_tool("pen")
move.j([settings],start=True) # moveing robot to point
#move.plot()
move.send_traj()
move.status("Done with operation_point_move.py")
