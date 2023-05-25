import jaco2
import paho.mqtt.client as mqtt
import numpy as np
import operation_line_citron 
import move
data = jaco2.GeneralInformations()
deg = np.pi/180
rad = 180/np.pi

client_data = mqtt.Client()
client_data.connect("test.mosquitto.org", 1883, 60)
def get_data():
    # Get data
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
    force6 = data.force_actuator6
    #temp1 = data.temperature1
    #temp2 = data.temperature2
    #temp3 = data.temperature3
    #temp4 = data.temperature4
    #temp5 = data.temperature5
    #temp6 = data.temperature6

    # Publish data
    # Cartisean
    client_data.publish("P2/AAU/current/x", x)
    client_data.publish("P2/AAU/current/y", y)
    client_data.publish("P2/AAU/current/z", z)
    client_data.publish("P2/AAU/current/roll", roll)
    client_data.publish("P2/AAU/current/pitch", pitch)
    client_data.publish("P2/AAU/current/yaw", yaw)
    # Power
    client_data.publish("P2/AAU/current/power", power)
    # Actuators
    client_data.publish("P2/AAU/current/theta1", act1)
    client_data.publish("P2/AAU/current/theta2", act2)
    client_data.publish("P2/AAU/current/theta3", act3)
    client_data.publish("P2/AAU/current/theta4", act4)
    client_data.publish("P2/AAU/current/theta5", act5)
    client_data.publish("P2/AAU/current/theta6", act6)
    client_data.publish("P2/AAU/current/finger1", fin1)
    client_data.publish("P2/AAU/current/finger2", fin2)
    client_data.publish("P2/AAU/current/finger3", fin3)
    # Force
    client_data.publish("P2/AAU/current/force1", force1)
    client_data.publish("P2/AAU/current/force2", force2)
    client_data.publish("P2/AAU/current/force3", force3)
    client_data.publish("P2/AAU/current/force4", force4)
    client_data.publish("P2/AAU/current/force5", force5)
    client_data.publish("P2/AAU/current/force6", force6)
    # Temperature
    #client_data.publish("P2/AAU/current/temp1", temp1)
    #client_data.publish("P2/AAU/current/temp2", temp2)
    #client_data.publish("P2/AAU/current/temp3", temp3)
    #client_data.publish("P2/AAU/current/temp4", temp4)
    #client_data.publish("P2/AAU/current/temp5", temp5)
    #client_data.publish("P2/AAU/current/temp6", temp6)

client_op = mqtt.Client()
client_op.connect("test.mosquitto.org", 1883, 60)
def operation():
    ready = True
    def on_connect(client, userdata, flags, rc):
        client.subscribe("P2/AAU/op")
    def on_message(client, userdata, msg):
        payload_str = msg.payload.decode("utf-8")
        print(f"Running operation: {payload_str}")
        nonlocal ready
        if ready:
            ready = False
            p = None
            if payload_str == "citron":
                operation_line_citron.operation_citron()
            elif payload_str == "line":
                operation_line_citron.operation_line()
            else: 
                print(f"Input recived: {payload_str}")
            ready = True
    client_op.on_connect = on_connect
    client_op.on_message = on_message
    client_op.loop_start()

client_point = mqtt.Client()
client_point.connect("test.mosquitto.org", 1883, 60)
def point_move():
    print("Started point move listener")
    x,y,z,roll,pitch,yaw = 0,0,0,0,0,0
    x_i,y_i,z_i,roll_i,pitch_i,yaw_i = 0,0,0,0,0,0
    def on_connect(client, userdata, flags, rc):
        client_point.subscribe("P2/AAU/goal/1")
        client_point.subscribe("P2/AAU/goal/2")
        client_point.subscribe("P2/AAU/goal/3")
        client_point.subscribe("P2/AAU/goal/4")
        client_point.subscribe("P2/AAU/goal/5")
        client_point.subscribe("P2/AAU/goal/6")
    def on_message(client, userdata, msg):
        nonlocal x,y,z,roll,pitch,yaw
        nonlocal x_i,y_i,z_i,roll_i,pitch_i,yaw_i
        payload_str = msg.payload.decode("utf-8")
        if msg.topic == "P2/AAU/goal/1":
            x = float(payload_str)
            x_i += 1
        elif msg.topic == "P2/AAU/goal/2":
            y = float(payload_str)
            y_i += 1
        elif msg.topic == "P2/AAU/goal/3":
            z = float(payload_str)
            z_i += 1
        elif msg.topic == "P2/AAU/goal/4":
            roll = float(payload_str)
            roll_i += 1
        elif msg.topic == "P2/AAU/goal/5":
            pitch = float(payload_str)
            pitch_i += 1
        elif msg.topic == "P2/AAU/goal/6":
            yaw = float(payload_str)
            yaw_i += 1
        if (x_i == y_i == z_i == roll_i == pitch_i == yaw_i) and (x_i != 0):
            x_i,y_i,z_i,roll_i,pitch_i,yaw_i = 0,0,0,0,0,0
            print(f"Moving to [{x},{y},{z},{roll},{pitch},{yaw}]")
            print(np.array([[x,y,z,roll,pitch,yaw]]))
            joint, vel = move.j(np.array([x,y,z,roll,pitch,yaw]),12*deg,12*deg)
            move.send(joint, vel)
    client_point.on_connect = on_connect
    client_point.on_message = on_message
    client_point.loop_start()


