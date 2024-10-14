#!/usr/bin/env python3

import os, random
import yaml
from pprint import pprint
from time import sleep

# MQTT handling
import paho.mqtt.client as mqtt
import json, msgpack

# ROS handling
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# Msg handling
import importlib
from std_msgs.msg import String, Empty
from vrviz.rosmsg import convert_ros_message_to_dictionary, get_rosmsg_obj

# Define topics to connect with
class FarmConnector(Node):
    def __init__(self):
        """ Initialise Handler """
        super().__init__('mqtt_server')
        self.name = os.getenv('VRVIZ_MQTT_CLIENT_NAME')

        # Define all the details for the MQTT broker
        self.mqtt_ip = os.getenv('VRVIZ_MQTT_BROKER_IP')
        self.mqtt_port = int(os.getenv('VRVIZ_MQTT_BROKER_PORT'))
        self.mqtt_ns = os.getenv('VRVIZ_MQTT_BROKER_NAMESPACE')
        print(f'Connecting as {self.name} to {self.mqtt_ip}:{self.mqtt_port} under namespace: {self.mqtt_ns}')
        if not self.mqtt_ns:
            print('VRVIZ_MQTT_BROKER_NAMESPACE envvar is empty, quitting.')
            quit()


        # Acquire Config Files
        self.rviz_config = os.getenv('VRVIZ_TABLE_CONFIG')
        if not self.rviz_config:
            print('VRVIZ_TABLE_CONFIG envvar is empty, quitting.')
            quit()
        self.config = dict()
        self.load_config()

        # Initiate connections to ROS and MQTT
        self.mqtt_subscribers = []
        self.connect_to_mqtt()

        # Manual Intervention
        self.subs = []
        self.create_subscription(Empty, '/load_config', self.load_config, 10)

    def connect_to_mqtt(self):
        """ MQTT management functions """
        self.mqtt_client = mqtt.Client(self.name)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        try:
            print('Connecting to broker...')
            self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
        except:
            print('Connection failed, attempting reconnect...')
            sleep(1)
            self.connect_to_mqtt()
            return
        print('Connection established.\n')
        self.mqtt_client.loop_start()


    def on_message(self, client, userdata, msg):
        """ Event callback trigger when mqtt subscription recieves a message """
        return

    def on_connect(self, client, userdata, flags, rc):
        """ Event callback trigger when mqtt establishes a connection """
        self.send_config()

    def load_config(self, msg=None):
        """ Load the config file """
        with open(self.rviz_config, 'r') as f:
            self.config['Table'] = yaml.safe_load(f.read())
        if msg != None:
            self.send_config()

    def send_config(self):
        """ Update and publish config to mqtt """
        # Load and publish the config
        self.config['Table']['Visualization Manager']['Displays'][0]['Alpha'] = random.random()

        # We have a new config potentially, so we should create any ros subscribers and publishers here now
        self.config['Table']['Panels'] = []
        self.config['Table']['Window Geometry'] = []
        D = self.config['Table']['Visualization Manager']['Displays']

        # Flatten Groups into a list of Dipslays (TODO: rework this to retain the heirarchy somehow)
        from pprint import pprint
        while any([d['Class'] == 'rviz_common/Group' for d in D]):
            D = [d for d in D if d['Class'] != 'rviz_common/Group'] + \
                sum([d['Displays'] for d in D if d['Class'] == 'rviz_common/Group'],[])

        #
        D = [d for d in D if d['Value'] == True]
        self.config['Table']['Visualization Manager']['Displays'] = D

        # Publish config for Table
        rviz_types = {#'rviz_default_plugins/Path': 'nav_msgs/msg/Path',
                        #'rviz_default_plugins/MarkerArray': 'visualization_msgs/msg/MarkerArray',
                        'rviz_default_plugins/Odometry':'nav_msgs/msg/Odometry',
                        'rviz_default_plugins/Pose':'geometry_msgs/msg/PoseStamped',
                        'rviz_default_plugins/PointStamped':'geometry_msgs/msg/PointStamped',
                        'rviz_default_plugins/PoseWithCovariance':'geometry_msgs/msg/PoseWithCovarianceStamped'}
        
        # Filter classes which have not been implemented yet
        self.config['Table']['Visualization Manager']['Displays'] = [
                t for t in self.config['Table']['Visualization Manager']['Displays'] 
                if t['Class'] in rviz_types.keys()
        ]

        self.mqtt_client.publish('vrviz/META', json.dumps(self.config['Table']), retain=True)

        for topic in self.config['Table']['Visualization Manager']['Displays']:

            # Handle if Topic does not exist
            if 'Topic' not in topic:
                print('Display class (', topic['Class'], ') does not use a topic.')
                continue

            print('')
            print('Topic name:', topic['Topic']['Value'])

            # Skip if display is disabled
            if topic['Value'] == False:
                print('|', 'disabled')
                continue
            if topic['Topic']['Value'] == '/topomap_marker2/vis':
                print('|', 'topo disabled')
                continue


            # Convert qos details to objects
            T = topic['Topic']
            R = {'Reliable': ReliabilityPolicy.RELIABLE, 'Best Effort': ReliabilityPolicy.BEST_EFFORT}
            H = {'Keep Last': HistoryPolicy.KEEP_LAST, 'Keep All': HistoryPolicy.KEEP_ALL}
            D = {'Volatile': DurabilityPolicy.VOLATILE, 'Transient Local': DurabilityPolicy.TRANSIENT_LOCAL}

            # Define Topic Details
            topic_name = T['Value']
            depth = T['Depth'] if 'Depth' in T else 1
            filter_size = T['Filter Size'] if 'Filter Size' in T else 1
            r = R[T['Reliability Policy'] if 'Reliability Policy' in T else 'Reliable']
            h = H[T['History Policy'] if 'History Policy' in T else 'Keep Last']
            d = D[T['Durability Policy'] if 'Durability Policy' in T else 'Volatile']

            # Skip if already connected
            if topic_name in self.subs:
                print('|', 'already connected')
                continue
            self.subs += [topic_name]

            # Get the message type
            #topics = self.get_publishers_info_by_topic(topic_name)
            #if len(topics) == 0:
            #    print('Topic is not publishing, so idk what to do... we need a data type...')
            #    continue
            #else:
            #    msg_type = topics[0][1][0]

            if topic['Class'] not in rviz_types:
                print('| ERROR, topic message type not defined: '+topic['Class'])
                continue

            module_name, class_name = rviz_types[topic['Class']].split('/msg/')
            rosmsg_type = getattr(importlib.import_module(module_name+".msg"), class_name)
            #print('|', topic['Class'])
            #print('|', rviz_types[topic['Class']])

            # Create ROS Subscriber
            qos = QoSProfile(depth=depth, reliability=r, history=h, durability=d)
            self.create_subscription(rosmsg_type, topic_name, lambda msg, t=topic_name: self.ros_cb(msg, t), qos)
            print('|', 'subscribed')

    def ros_cb(self, msg, topic):
        """ ROS Callback finction to service all ROS subscribers """
        print('')
        print('|','ROS Message recieved: [' + topic + ']')

        # Encode msg to JSON
        data = json.dumps(convert_ros_message_to_dictionary(msg))
        print(data[:50]+'...' if len(data)>50 else data)

        # Publish msg to mqtt
        print(topic)
        print(self.mqtt_ns)
        mqtttopic = self.mqtt_ns + topic
        self.mqtt_client.publish(mqtttopic, data, retain=False)


def main(args=None):
    rclpy.init(args=args)

    FC = FarmConnector()
    rclpy.spin(FC)

    FC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

