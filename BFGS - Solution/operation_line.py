import move
import numpy as np
from MQTT import MqttClient
from Jaco2_erobot import KinovaJaco2
robot = KinovaJaco2()

mqtt_client = MqttClient()
deg = np.pi/180
rad = 180/np.pi

def calculate_lines(settings):
    move.status("Calculating trajectory...")
    # Define the two points
    point1 = np.array([settings[0], settings[1]])
    point2 = np.array([settings[2], settings[3]])
    print(f"point1: {point1}")
    print(f"point2: {point2}")

    # Calculate the direction vector between the two points
    direction_vector = point2 - point1

    # Calculate the perpendicular vector to the direction vector
    perpendicular_vector = np.array([-direction_vector[1], direction_vector[0]])

    # Normalize the perpendicular vector
    perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)

    #Define the offset
    offset = 25

    # Calculate the offset vector
    offset_vector = perpendicular_vector * offset
    # Calculate the new points
    point3 = point1 + offset_vector
    point4 = point2 + offset_vector

    # Calculate the offset vector
    offset_vector = perpendicular_vector * (-offset)
    # Calculate the new points
    point5 = point1 + offset_vector
    point6 = point2 + offset_vector

    # Construct the line segments
    line2 = np.array([point1, point2])
    line1 = np.array([point3, point4])
    line3 = np.array([point5, point6])
    print(f"line1: {line1}")
    print(f"line2: {line2}")
    print(f"line3: {line3}")
    return line1, line2, line3

move.status("Starting operation_line.py")
move.status("Waiting for coords...")
operation_setting_topics = [
    "P2/AAU/coords/1", # x1
    "P2/AAU/coords/2", # y1
    "P2/AAU/coords/3", # x2
    "P2/AAU/coords/4"] # y2
settings = mqtt_client.subscribe_and_wait_for_messages(operation_setting_topics)
#settings = np.array([0,0,100,100])
move.status("Coords recived!")

# Calculate lines
line1, line2, line3 = calculate_lines(settings)

# Define trajectories
move.change_tool("pen")
move.j([[-200,-100,120],[0,0,120]],start=True) # moveing robot to work space
# first line
move.l([[*line1[0],30],[*line1[0],0]])
move.l([[*line1[0],0],[*line1[1],0]]) # draw
move.l([[*line1[1],0],[*line1[1],30]])
# second line
move.l([[*line2[0],30],[*line2[0],0]])
move.l([[*line2[0],0],[*line2[1],0]]) # draw
move.l([[*line2[1],0],[*line2[1],30]])
# third line
move.l([[*line3[0],30],[*line3[0],0]])
move.l([[*line3[0],0],[*line3[1],0]]) # draw
move.l([[*line3[1],0],[*line3[1],30]])
# going back to zero position
move.j([[0,0,120]]) 

move.plot()
move.send_traj()
move.status("Done with operation_line.py!")