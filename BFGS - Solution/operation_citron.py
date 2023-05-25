import numpy as np
import matplotlib.pyplot as plt
import move
from Jaco2_erobot import KinovaJaco2
robot = KinovaJaco2()
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180
rad = 180/np.pi

class Citron:
    def __init__(self):
        pass

    def p2a(self, p1, p2):
        x_array = np.array([p1[0], p2[0]])
        y_array = np.array([p1[1], p2[1]])
        return x_array, y_array

    def citron(self,ptk1, ptk2, inverted = False):
        x1 = ptk1[0]
        y1 = ptk1[1]
        x2 = ptk2[0]
        y2 = ptk2[1]
        t = 0.5
        A = np.array([x1, y1])
        B = np.array([x2, y2])
        pointBA = np.array((A-B))*t+B     #til t=0 har man punktet B og til t=1 har man punktet A, dermed er centerpunktet ved t=0.5
        
        angle = np.arctan2(y1-y2, x1-x2)*180/np.pi
        
        #print(f"angle:  {angle}")
        x = np.linspace(0, 2*np.pi, 100)
        center=pointBA
        radius = 103 #radius of the citron is 10.3 cm but i want the mid points
        if inverted:
            radius = -radius
        point1 = np.array([radius*np.cos((-30+angle)*np.pi/180)+center[0],  radius*np.sin((-30+angle)*np.pi/180)+center[1]])
        #print(point1)
        point2 = np.array([radius*np.cos((-60+angle)*np.pi/180)+center[0], radius*np.sin((-60+angle)*np.pi/180)+center[1]])
        #print(point2)
        point3 = np.array([radius*np.cos((-90+angle)*np.pi/180)+center[0], radius*np.sin((-90+angle)*np.pi/180)+center[1]])
        #print(point3)
        point4 = np.array([radius*np.cos((-120+angle)*np.pi/180)+center[0], radius*np.sin((-120+angle)*np.pi/180)+center[1]])
        #print(point4)
        point5 = np.array([radius*np.cos((-150+angle)*np.pi/180)+center[0], radius*np.sin((-150+angle)*np.pi/180)+center[1]])
        #print(point5)
        #calculate orientation
        ori1 = (-30+angle)*np.pi/180
        ori2 = (-60+angle)*np.pi/180
        ori3 = (-90+angle)*np.pi/180
        ori4 = (-120+angle)*np.pi/180
        ori5 = (-150+angle)*np.pi/180
        orientationArray = np.array([ori1, ori2, ori3, ori4, ori5])
        """ # Plot lines------------------------------------------------
        x,y = self.p2a(B, A)
        x1,y1 = self.p2a(center, point1)
        x2,y2 = self.p2a(center, point2)
        x3,y3 = self.p2a(center, point3)
        x4,y4 = self.p2a(center, point4)
        x5,y5 = self.p2a(center, point5)
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')
        # Plot lines
        ax.plot(x, y, label='0')
        ax.plot(x1, y1, label='1')
        ax.plot(x2, y2, label='2')
        ax.plot(x3, y3, label='3')
        ax.plot(x4, y4, label='4')
        ax.plot(x5, y5, label='5')
        # Add legend
        plt.legend()
        # Show plot
        plt.show() """
        pointArray = np.array([point1, point2, point3, point4, point5])
        return center, orientationArray

move.status("Starting operation_citron.py")

# Get points
move.status("Waiting for coords...")
operation_setting_topics = [
    "P2/AAU/coords/1", # x1
    "P2/AAU/coords/2", # y1
    "P2/AAU/coords/3", # x2
    "P2/AAU/coords/4", # y2
    "P2/AAU/coords/5"] # flipped
settings = mqtt_client.subscribe_and_wait_for_messages(operation_setting_topics)
move.status("Coords recived!")
#settings = [50,60,50,260,True]

# Define the two points
point1 = np.array([settings[0], settings[1]])
point2 = np.array([settings[2], settings[3]])
print(f"point1: {point1}")
print(f"point2: {point2}")
flipped = bool(settings[4])
print(f"flipped: {flipped}")
citron_hight = 60

# Calculate cutting positions
C = Citron()
center, orientation = C.citron(point1,point2,flipped)

# Define trajectories
move.change_tool("knife")
move.j([[-200,-100,120],[0,0,120]],start=True) # moveing robot to work space
for i, o in enumerate(orientation):
    move.l([[*center,citron_hight,0,0,o],[*center,0,0,0,o]]) # cutting down
    move.l([[*center,0,0,0,o],[*center,citron_hight,0,0,o]]) # moving up
move.j([[0,0,120]]) # going back to zero position
move.plot()
move.send_traj()
move.status("Done with operation_citron.py")