from flask import Flask, render_template, jsonify
from flask import Flask, request

import paho.mqtt.client as mqtt

app = Flask(__name__)



# MQTT broker indstillinger.
broker_address = "test.mosquitto.org"
broker_port = 1883
topics = ["P2/AAU/current/theta1","P2/AAU/current/theta2","P2/AAU/current/theta3","P2/AAU/current/theta4","P2/AAU/current/theta5","P2/AAU/current/theta6","P2/AAU/current/x","P2/AAU/current/y","P2/AAU/current/z","P2/AAU/current/roll","P2/AAU/current/pitch","P2/AAU/current/yaw","P2/AAU/goal/1", "P2/AAU/goal/2", "P2/AAU/goal/3", "P2/AAU/goal/4", "P2/AAU/goal/5", "P2/AAU/goal/6","P2/AAU/goalAngles/1", "P2/AAU/goalAngles/2", "P2/AAU/goalAngles/3", "P2/AAU/goalAngles/4", "P2/AAU/goalAngles/5", "P2/AAU/goalAngles/6","P2/AAU/current/power","P2/AAU/current/finger1","P2/AAU/current/finger2","P2/AAU/current/finger3","P2/AAU/current/force1","P2/AAU/current/force2","P2/AAU/current/force3","P2/AAU/current/force4","P2/AAU/current/force5","P2/AAU/current/force6","P2/AAU/current/temp1","P2/AAU/current/temp2","P2/AAU/current/temp3","P2/AAU/current/temp4","P2/AAU/current/temp5","P2/AAU/current/temp6","P2/AAU/coords/cx1","P2/AAU/coords/cy1","P2/AAU/coords/cx2","P2/AAU/coords/cy2","P2/AAU/coords/flip","P2/AAU/info"]



# Create MQTT client
client = mqtt.Client()



def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribe til topic ved conncetion
    for topic in topics:
        client.subscribe(topic)



# variabler til at gemme seneste ændret global
theta1 = ""
theta2 = ""
theta3 = ""
theta4 = ""
theta5 = ""
theta6 = ""
Cx = ""
Cy = ""
Cz = ""
Croll =""
Cpitch = ""
Cyaw = ""
info = ""

# indkommende data
def on_message(client, userdata, msg):
    global C_theta1
    global C_theta2
    global C_theta3
    global C_theta4
    global C_theta5
    global C_theta6
    global C_force1
    global C_force2
    global C_force3
    global C_force4
    global C_force5
    global C_force6
    global C_finger1
    global C_finger2
    global C_finger3
    global C_temp1
    global C_temp2
    global C_temp3
    global C_temp4
    global C_temp5
    global C_temp6
    global C_x
    global C_y
    global C_z
    global C_roll
    global C_pitch
    global C_yaw
    global C_power
    global goal_angle_x
    global goal_angle_y
    global goal_angle_z
    global goal_angle_roll
    global goal_angle_pitch
    global goal_angle_yaw
    global goal_x
    global goal_y
    global goal_z
    global goal_roll
    global goal_pitch
    global goal_yaw
    global cx1
    global cx2
    global cy1
    global cy2
    global info
    if msg.topic=="P2/AAU/current/theta1":
        C_theta1 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/theta2":
        C_theta2 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/theta3":
        C_theta3 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/theta4":
        C_theta4 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/theta5":
        C_theta5 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/theta6":
        C_theta6 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/x":
        C_x = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/y":
        C_y = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/z":
        C_z = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/roll":
        C_roll = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/pitch":
        C_pitch = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/yaw":
        C_yaw = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/power":
        C_power = msg.payload.decode()
    elif msg.topic=="P2/AAU/goalAngles/1":
        goal_angle_x = msg.payload.decode()
    elif msg.topic=="P2/AAU/goalAngles/2":
        goal_angle_y = msg.payload.decode()
    elif msg.topic=="P2/AAU/goalAngles/3":
        goal_angle_z = msg.payload.decode()
    elif msg.topic=="P2/AAU/goalAngles/4":
        goal_angle_roll = msg.payload.decode()
    elif msg.topic=="P2/AAU/goalAngles/5":
        goal_angle_pitch = msg.payload.decode()
    elif msg.topic=="P2/AAU/goalAngles/6":
        goal_angle_yaw = msg.payload.decode()
    elif msg.topic=="P2/AAU/goal/1":
        goal_x = msg.payload.decode()
    elif msg.topic=="P2/AAU/goal/2":
        goal_y = msg.payload.decode()
    elif msg.topic=="P2/AAU/goal/3":
        goal_z = msg.payload.decode()
    elif msg.topic=="P2/AAU/goal/4":
        goal_roll = msg.payload.decode()
    elif msg.topic=="P2/AAU/goal/5":
        goal_pitch = msg.payload.decode()
    elif msg.topic=="P2/AAU/goal/6":
        goal_yaw = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/force1":
        C_force1 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/force2":
        C_force2 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/force3":
        C_force3 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/force4":
        C_force4 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/force5":
        C_force5 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/force6":
        C_force6 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/finger1":
        C_finger1 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/finger2":
        C_finger2 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/finger3":
        C_finger3 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/temp1":
        C_temp1 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/temp2":
        C_temp2 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/temp3":
        C_temp3 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/temp4":
        C_temp4 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/temp5":
        C_temp5 = msg.payload.decode()
    elif msg.topic=="P2/AAU/current/temp6":
        C_temp6 = msg.payload.decode()
    elif msg.topic=="P2/AAU/coords/cx1":
        cx1 = msg.payload.decode()
    elif msg.topic=="P2/AAU/coords/cy1":
        cy1 = msg.payload.decode()
    elif msg.topic=="P2/AAU/coords/cx2":
        cx2 = msg.payload.decode()
    elif msg.topic=="P2/AAU/coords/cy2":
        cy2 = msg.payload.decode()
    elif msg.topic=="P2/AAU/info":
        info = msg.payload.decode()
    
    print(f"{msg.topic} {msg.payload.decode()}")      

client.on_connect = on_connect
client.on_message = on_message

# Connect yil MQTT broker
client.connect(broker_address, broker_port, 60)

# start MQTT client loop
client.loop_start()


@app.route('/C_x')
def get_dataC_x():
    global C_x
    return C_x

@app.route('/C_y')
def get_dataC_y():
    global C_y
    return C_y

@app.route('/C_z')
def get_dataC_z():
    global C_z
    return C_z

@app.route('/C_roll')
def get_dataC_roll():
    global C_roll
    return C_roll

@app.route('/C_pitch')
def get_dataC_pitch():
    global C_pitch
    return C_pitch

@app.route('/C_yaw')
def get_dataC_yaw():
    global C_yaw
    return C_yaw

@app.route('/C_theta1')
def get_dataC_theta1():
    global C_theta1
    return C_theta1

@app.route('/C_theta2')
def get_dataC_theta2():
    global C_theta2
    return C_theta2

@app.route('/C_theta3')
def get_dataC_theta3():
    global C_theta3
    return C_theta3

@app.route('/C_theta4')
def get_dataC_theta4():
    global C_theta4
    return C_theta4

@app.route('/C_theta5')
def get_dataC_theta5():
    global C_theta5
    return C_theta5

@app.route('/C_theta6')
def get_dataC_theta6():
    global C_theta6
    return C_theta6

@app.route('/C_power')
def get_dataC_power():
    global C_power
    return C_power

@app.route('/C_force1')
def get_dataC_force1():
    global C_force1
    return C_force1

@app.route('/C_force2')
def get_dataC_force2():
    global C_force2
    return C_force2

@app.route('/C_force3')
def get_dataC_force3():
    global C_force3
    return C_force3

@app.route('/C_force4')
def get_dataC_force4():
    global C_force4
    return C_force4

@app.route('/C_force5')
def get_dataC_force5():
    global C_force5
    return C_force5

@app.route('/C_force6')
def get_dataC_force6():
    global C_force6
    return C_force6

@app.route('/C_finger1')
def get_dataC_finger1():
    global C_finger1
    return C_finger1

@app.route('/C_finger2')
def get_dataC_finger2():
    global C_finger2
    return C_finger2

@app.route('/C_finger3')
def get_dataC_finger3():
    global C_finger3
    return C_finger3
@app.route('/C_temp1')
def get_dataC_temp1():
    global C_temp1
    return C_temp1

@app.route('/C_temp2')
def get_dataC_temp2():
    global C_temp2
    return C_temp2

@app.route('/C_temp3')
def get_dataC_temp3():
    global C_temp3
    return C_temp3

@app.route('/C_temp4')
def get_dataC_temp4():
    global C_temp4
    return C_temp4

@app.route('/C_temp5')
def get_dataC_temp5():
    global C_temp5
    return C_temp5

@app.route('/C_temp6')
def get_dataC_temp6():
    global C_temp6
    return C_temp6

@app.route('/info')
def get_datainfo():
    global info
    return info

#defining global xyz rpy variables
x = ""
y = ""
z = ""
roll = ""
pitch = ""
yaw = ""

@app.route('/goal', methods=['POST'])
def receive_goal():
    data = request.json
    x = data['x']
    y = data['y']
    z = data['z']
    roll = data['roll']
    pitch = data['pitch']
    yaw = data['yaw']
    broker_address = "test.mosquitto.org"  # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    messages = [x,y,z,roll,pitch,yaw]
    i = 0
    for message in messages:
        i = i+1
        topic = f"P2/AAU/goal/{i}"
        #UD kommateret, ved fejlfinden fjern #
        #print(topic)
        client.publish(topic, message)
    return 'Received goal: x={}, y={}, z={}, roll={}, pitch={}, yaw={}'.format(x, y, z, roll, pitch, yaw)
 
 #Goal Joint Angles
@app.route('/goal2', methods=['POST'])
def receive_goal2():
    data = request.json
    Theta1 = data['Theta1']
    Theta2 = data['Theta2']
    Theta3 = data['Theta3']
    Theta4 = data['Theta4']
    Theta5 = data['Theta5']
    Theta6 = data['Theta6']
    
    broker_address = "test.mosquitto.org"  # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    messages = [Theta1,Theta2,Theta3,Theta4,Theta5,Theta6]
    i = 0
    for message in messages:
        i = i+1
        topic = f"P2/AAU/goalAngles/{i}"
        #UD kommateret, ved fejlfinden fjern #
        #print(topic)
        client.publish(topic, message)
    return 'Received goal: Theta1={}, Theta2={}, Theta3={}, Theta4={}, Theta5={}, Theta6={}'.format(Theta1,Theta2,Theta3,Theta4,Theta5,Theta6)


@app.route('/coords', methods=['POST'])
def coords():
    data = request.json
    cx1 = data['cx1']
    cy1 = data['cy1']
    cx2 = data['cx2']
    cy2 = data['cy2']
    flip = data['flipValue']


    broker_address = "test.mosquitto.org"   # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    messages = [cx1,cy1,cx2,cy2,flip]
    i = 0
    for message in messages:
        i = i+1
        topic = f"P2/AAU/coords/{i}"
        #UD kommateret, ved fejlfinden fjern #
        #print(topic)
        client.publish(topic, message)
    return 'coords: cx1={}, cy1={}, cx2={}, cy2={}, flip={}'.format(cx1,cy1,cx2,cy2,flip)

@app.route('/citron', methods=['POST'])
def receive_citron():
    data = request.json
    msg = data['msg']
    

    broker_address = "test.mosquitto.org"  # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    
    client.publish("P2/AAU/op", msg)
    return 'Received goal: msg={}'.format(msg)


@app.route('/line', methods=['POST'])
def receive_line():
    data = request.json
    msg = data['msg']
    

    broker_address = "test.mosquitto.org"  # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    client.publish("P2/AAU/op", msg)
    return 'Received goal: msg={}'.format(msg)

@app.route('/accept', methods=['POST'])
def acceptAction():
    data = request.json
    msg = data['msg']
    

    broker_address = "test.mosquitto.org"  # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    
    client.publish("P2/AAU/op", msg)
    return 'Received goal: msg={}'.format(msg)

@app.route('/decline', methods=['POST'])
def declineAction():
    data = request.json
    msg = data['msg']
    

    broker_address = "test.mosquitto.org"  # MQTT broker adresse her
    client = mqtt.Client()
    client.connect(broker_address)
    
    client.publish("P2/AAU/op", msg)
    return 'Received goal: msg={}'.format(msg)







# Home page route
@app.route('/')
def home():
    return render_template('index.html')




if __name__ == '__main__':
    app.run(debug=False)