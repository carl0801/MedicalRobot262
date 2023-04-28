import paho.mqtt.client as mqtt
import time
emty = {}

class MqttClient:
    def __init__(self):
        # Set up the MQTT client
        self.client = mqtt.Client()
        self.client.connect("test.mosquitto.org", 1883, 60)

        # Define a dictionary to store the messages received for each topic
        self.messages = {}

    # Define a function to handle incoming MQTT messages
    def on_message(self, client, userdata, message):
        # Extract the topic and message value from the MQTT message
        topic = message.topic
        value = float(message.payload.decode())

        # Store the message value for the topic in the messages dictionary
        self.messages[topic] = value

    def subscribe_and_wait_for_messages(self, topics):
        
        # Subscribe to the specified MQTT topics and wait for messages with values > 0
        for topic in topics:
            self.client.subscribe(topic)
        t = time.time()
        while t+0.01 > time.time():
            # Check if all messages have been received
            if all(topic in self.messages and self.messages[topic] != emty for topic in topics):
                # Store the messages in a local variable
                values = [self.messages[topic] for topic in topics]
                self.messages = {}
                return values
            # Wait for incoming MQTT messages
            self.client.on_message = self.on_message
            self.client.loop(timeout=1)

        return None

    def publish_message(self, topic, message):
        self.client.publish(topic, message)

