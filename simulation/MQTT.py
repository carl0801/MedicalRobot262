import paho.mqtt.client as mqtt

empty = {}

class MqttClient:
    def __init__(self):
        # Set up the MQTT client
        self.client = mqtt.Client()
        self.client.connect("test.mosquitto.org", 1883, 60)

        # Define the MQTT topics to subscribe to
        self.topics = [
            "P2/AAU/CURRENT/theta1",
            "P2/AAU/CURRENT/theta2",
            "P2/AAU/CURRENT/theta3",
            "P2/AAU/CURRENT/theta4",
            "P2/AAU/CURRENT/theta5",
            "P2/AAU/CURRENT/theta6",
        ]

        # Define a dictionary to store the messages received for each topic
        self.messages = {}

    # Define a function to handle incoming MQTT messages
    def on_message(self, client, userdata, message):
        # Extract the topic and message value from the MQTT message
        topic = message.topic
        value = float(message.payload.decode())

        # Store the message value for the topic in the messages dictionary
        self.messages[topic] = value

    def subscribe_and_wait_for_messages(self):
        # Subscribe to the MQTT topics and wait for messages with values > 0
        for topic in self.topics:
            self.client.subscribe(topic)

        while True:
            # Check if all messages have been received
            if all(topic in self.messages != empty for topic in self.topics):
                # Store the messages in a local variable
                values = [self.messages[topic] for topic in self.topics]
                self.messages = {}
                return values

            # Wait for incoming MQTT messages
            self.client.on_message = self.on_message
            self.client.loop(timeout=1)

