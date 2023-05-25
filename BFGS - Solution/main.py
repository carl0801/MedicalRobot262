import jaco2
import asyncio
import signal
import sys
import move
import numpy as np
import paho.mqtt.client as mqtt
import subprocess
from MQTT import MqttClient
mqtt_client = MqttClient()
deg = np.pi/180

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
    temp1 = data.temperature1
    temp2 = data.temperature2
    temp3 = data.temperature3
    temp4 = data.temperature4
    temp5 = data.temperature5
    temp6 = data.temperature6

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
    client_data.publish("P2/AAU/current/temp1", temp1)
    client_data.publish("P2/AAU/current/temp2", temp2)
    client_data.publish("P2/AAU/current/temp3", temp3)
    client_data.publish("P2/AAU/current/temp4", temp4)
    client_data.publish("P2/AAU/current/temp5", temp5)
    client_data.publish("P2/AAU/current/temp6", temp6)

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
                p = subprocess.Popen(["python", "operation_citron.py"])
            elif payload_str == "line":
                p = subprocess.Popen(["python", "operation_line.py"])
            elif payload_str == "point":
                p = subprocess.Popen(["python", "operation_point_move.py"])
            if p is not None:  # check if p is not None
                p.communicate()  # wait for the process to finish
            else: 
                print(f"Input recived: {payload_str}")
            ready = True
    client_op.on_connect = on_connect
    client_op.on_message = on_message
    client_op.loop_start()

def stop_handler(signal, frame):
    print("\nStopping program...")
    move.status("Stopped robot connection!")
    for task in asyncio.all_tasks():
        task.cancel()
    jaco2.close_robot()
    sys.exit(0)

signal.signal(signal.SIGINT, stop_handler)

async def update_data():
    print("Started async data update")
    while True:
        get_data()
        await asyncio.sleep(0.2)

async def start_operation():
    print("Started operation listener")
    while True:
        operation()
        await asyncio.sleep(0.2)

if __name__ == '__main__':
    try:
        move.status("Started robot connection!")
        loop = asyncio.get_event_loop()
        init_result = jaco2.init_robot()
        # if robot not connected
        if init_result != 1010:
            move.status("Moving Robot home!")
            move.home()
        print("Robot initialized")
        data_updater = loop.create_task(update_data())
        operation_starter = loop.create_task(start_operation())
        move.status("Robot ready for input!")
        loop.run_until_complete(asyncio.gather(data_updater, operation_starter))
    except KeyboardInterrupt:
        stop_handler(None, None)


