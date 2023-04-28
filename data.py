import jaco2
import paho.mqtt.client as mqtt
import numpy as np

data = jaco2.GeneralInformations()
broker_address = "test.mosquitto.org"  # replace with your MQTT broker's IP address or hostname
client = mqtt.Client()
client.connect(broker_address)

def get_data():
    """ # Get data
    data = jaco2.get_general_informations()
    # Set data
    x = data.cartesian_x*1000
    y = data.cartesian_y*1000
    z = data.cartesian_z*1000
    roll = data.cartesian_theta_x
    pitch = data.cartesian_theta_y
    yaw = data.cartesian_theta_z
    power = data.power
    act1 = data.actuator1
    act2 = data.actuator2
    act3 = data.actuator3
    act4 = data.actuator4
    act5 = data.actuator5
    act6 = data.actuator6
    fin1 = data.finger1
    fin2 = data.finger2 
    fin3 = data.finger3 
    force1 = data.force_actuator1
    force2 = data.force_actuator2
    force3 = data.force_actuator3
    force4 = data.force_actuator4
    force5 = data.force_actuator5
    force6 = data.force_actuator6 """

    # Set data test
    x = np.round(np.random.uniform(-1000, 1000),1)
    y = np.round(np.random.uniform(-1000, 1000),1)
    z = np.round(np.random.uniform(-1000, 1000),1)
    roll = np.round(np.random.uniform(-np.pi, np.pi),3)
    pitch = np.round(np.random.uniform(-np.pi, np.pi),3)
    yaw = np.round(np.random.uniform(-np.pi, np.pi),3)
    power = np.round(np.random.uniform(0, 100),2)
    act1 = np.round(np.random.uniform(-np.pi, np.pi),3)
    act2 = np.round(np.random.uniform(-np.pi, np.pi),3)
    act3 = np.round(np.random.uniform(-np.pi, np.pi),3)
    act4 = np.round(np.random.uniform(-np.pi, np.pi),3)
    act5 = np.round(np.random.uniform(-np.pi, np.pi),3)
    act6 = np.round(np.random.uniform(-np.pi, np.pi),3)
    fin1 = np.round(np.random.uniform(-np.pi, np.pi),3)
    fin2 = np.round(np.random.uniform(-np.pi, np.pi),3)
    fin3 = np.round(np.random.uniform(-np.pi, np.pi),3)
    force1 = np.round(np.random.uniform(0, 100),2)
    force2 = np.round(np.random.uniform(0, 100),2)
    force3 = np.round(np.random.uniform(0, 100),2)
    force4 = np.round(np.random.uniform(0, 100),2)
    force5 = np.round(np.random.uniform(0, 100),2)
    force6 = np.round(np.random.uniform(0, 100),2)
    # Publish data
    # Cartisean
    client.publish("P2/AAU/CURRENT/x", x)
    client.publish("P2/AAU/CURRENT/y", y)
    client.publish("P2/AAU/CURRENT/z", z)
    client.publish("P2/AAU/CURRENT/roll", roll)
    client.publish("P2/AAU/CURRENT/pitch", pitch)
    client.publish("P2/AAU/CURRENT/yaw", yaw)
    # Power
    client.publish("P2/AAU/CURRENT/power", power)
    # Actuators
    client.publish("P2/AAU/CURRENT/theta1", act1)
    client.publish("P2/AAU/CURRENT/theta2", act2)
    client.publish("P2/AAU/CURRENT/theta3", act3)
    client.publish("P2/AAU/CURRENT/theta4", act4)
    client.publish("P2/AAU/CURRENT/theta5", act5)
    client.publish("P2/AAU/CURRENT/theta6", act6)
    client.publish("P2/AAU/CURRENT/finger1", fin1)
    client.publish("P2/AAU/CURRENT/finger2", fin2)
    client.publish("P2/AAU/CURRENT/finger3", fin3)
    # Force
    client.publish("P2/AAU/CURRENT/force1", force1)
    client.publish("P2/AAU/CURRENT/force2", force2)
    client.publish("P2/AAU/CURRENT/force3", force3)
    client.publish("P2/AAU/CURRENT/force4", force4)
    client.publish("P2/AAU/CURRENT/force5", force5)
    client.publish("P2/AAU/CURRENT/force6", force6)

