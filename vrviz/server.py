#!/usr/bin/env python3

import os
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
from std_msgs.msg import String
from ffr_utils.rosmsg import convert_ros_message_to_dictionary, get_rosmsg_obj

# Define topics to connect with
class FarmConnector(Node):
    def __init__(self):
        """ Initialise Handler """
        super().__init__('farm_connector')
        self.source = 'field'
        self.name = os.getenv('FIELD_NAME')
        self.load_config()

        # Define all the details for the MQTT broker
        self.mqtt_ip = os.getenv('FARM_IP')
        self.mqtt_port = int(os.getenv('FARM_MQTT_BROKER_PORT'))

        # Initiate connections to ROS and MQTT
        self.ros_publishers = dict()
        self.ros_subscribers = dict()
        self.mqtt_subscribers = []
        self.connect_to_mqtt()


    def connect_to_mqtt(self):
        """ MQTT management functions """
        self.mqtt_client = mqtt.Client('field_'+self.name)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        try:
            print('Connecting to farm broker...')
            self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
        except:
            print('Connection failed, attempting reconnect...')
            sleep(1)
            self.connect_to_mqtt()
            return
        print('Connection established.')
        self.mqtt_client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        """ Event callback trigger when mqtt establishes a connection """
        pprint(self.config)
        self.mqtt_client.subscribe('REFRESH_FIELDS')

        topics_to_open = {k:v for k,v in self.config['TOPICS'].items() if v['target']=='field'}
        for k,v in topics_to_open.items():
            self.mqtt_client.subscribe(v['mqtt_namespace']+k)
            self.mqtt_subscribers += [v['mqtt_namespace']+k]

        self.send_config()


    def load_config(self):
        """ Load the config file """
        pkg = get_package_share_directory('ffr_field')
        self.topic_config = pkg+'/config/farm_field_connections.yaml'
        with open(self.topic_config, 'r') as f:
            self.config = yaml.safe_load(f.read())

    def send_config(self):
        """ Update and publish config to mqtt """
        # Load and publish the config
        self.load_config()
        self.mqtt_client.publish('META', json.dumps(self.config))

        # Attach default values to topic details
        for topic in self.config['TOPICS'].values():
            for default_key, default_value in self.config['GENERAL']['DEFAULTS'].items():
                if default_key not in topic:
                    topic[default_key] = default_value
        #self.config['TOPICS']['new_field']['target_namespace'] = '/bob/'

        # We have a new config potentially, so we should create any ros subscribers and publishers here now
        for topic_name, topic_details in self.config['TOPICS'].items():
            print('\n'*5)
            print('|','NEW TOPIC')
            print('|','name',topic_name)
            print('|','source',topic_details['source'])
            pprint(topic_details)
            print('')

            # If we are pubishing to the farm
            if topic_details['source'] == 'field':

                # Skip if already subscribed
                rostopic = topic_details['source_namespace'] + topic_name
                if rostopic in self.ros_subscribers.keys():
                    print('|   | skipping, already subscribed: ' + str(self.ros_subscribers.keys()))
                    continue

                # Get the message type
                module_name, class_name = topic_details['source_type_name'].split('/msg/')
                rosmsg_type = get_rosmsg_obj(module_name, class_name)

                # Create ros publisher and mqtt subscription
                qos = QoSProfile(depth=10)
                #if topic_details['source_latch']:
                #    r = ReliabilityPolicy.RELIABLE
                #    h = HistoryPolicy.KEEP_LAST
                #    d = DurabilityPolicy.TRANSIENT_LOCAL
                #    qos = QoSProfile(depth=10, reliability=r, history=h, durability=d)
                self.ros_subscribers[rostopic] = self.create_subscription(rosmsg_type, rostopic, lambda msg, topic=rostopic: self.ros_cb(msg, topic), qos)
                print('|   | creating rossub: ' + rostopic)

            # If we are subscribing to the farm
            if topic_details['target'] == 'field':


                # Skip rostopic if already publishing
                rostopic = topic_details['target_namespace']+topic_name
                if rostopic in self.ros_publishers.keys(): continue

                # Get the message type
                module_name, class_name = topic_details['target_type_name'].split('/msg/')
                rosmsg_type = getattr(importlib.import_module(module_name+".msg"), class_name)

                # Create ros publisher and mqtt subscription
                qos = QoSProfile(depth=10)
                #if topic_details['target_latch']:
                #    r = ReliabilityPolicy.RELIABLE
                #    h = HistoryPolicy.KEEP_LAST
                #    d = DurabilityPolicy.TRANSIENT_LOCAL
                #    qos = QoSProfile(depth=10, reliability=r, history=h, durability=d)
                print('|   | creating ros pub: ' + rostopic)
                self.ros_publishers[rostopic] = self.create_publisher(rosmsg_type, rostopic, qos)

                print('|   | creating mqtt pub: ' + mqtttopic)
                self.mqtt_client.subscribe(mqtttopic)
                self.mqtt_subscribers += [mqtttopic]

        # Remove publishers and subscribers no longer in use #TODO: doesnt work because of namespaces not being included
        #print('\n'*2)
        #print('Removing publisher and subscribers no longer in use')
        #print(self.config['TOPICS'].keys())
        #for pub in self.ros_publishers:
        #    print('|  rospub', pub)
        #    if pub not in self.config['TOPICS'].keys():
        #        print('|    | deleting rospub (skipped for now)')
        #        #del pub
        #for sub in self.ros_subscribers:
        #    print('|  rossub', sub)
        #    if sub not in self.config['TOPICS'].keys():
        #        print('|    | deleting rossub (skipping for now)')
        #        #del sub
        #for sub in self.mqtt_subscribers:
        #    print('| mqttsub', sub)
        #    if sub not in self.config['TOPICS'].keys():
        #        print('|    | deleting mqttsub (skipping for now)')
        #        #del sub

        print('')
        print('|','STATUS')
        print('|','Subscriptions to ROS:')
        print('|',', '.join(self.ros_subscribers.keys()) or '[]')
        print('|')
        print('|','Publishers to ROS:')
        print('|',', '.join(self.ros_publishers.keys()) or '[]')
        print('|')
        print('|','Subscriptions to MQTT:')
        print('|',', '.join(self.mqtt_subscribers) or '[]')
        print('|')

    def ros_cb(self, msg, rostopic):
        """ ROS Callback finction to service all ROS subscribers """
        print('')
        print('|','ROS Message recieved: [' + rostopic + ']')

        # Remap the topic details to a source_namespace dictionary
        source_topics = {v['source_namespace']+k:k for k,v in self.config['TOPICS'].items()}
        topic_name = source_topics[rostopic]
        topic = self.config['TOPICS'][topic_name]

        # Convert message to correct format
        if topic['broker_type_name'] == 'msgpack':
            # Bytearray is only used with msgpack encoding
            data = bytearray(msgpack.dumps(convert_ros_message_to_dictionary(msg)))
        elif topic['broker_type_name'] == 'json':
            data = json.dumps(convert_ros_message_to_dictionary(msg))
        print(data[:50]+'...' if len(data)>50 else data)

        # Publish msg to mqtt
        mqtttopic = topic['broker_namespace']+topic_name
        self.mqtt_client.publish(mqtttopic, data, retain=False)
        #self.mqtt_client.publish(mqtttopic, data, retain=topic['broker_latch'])

    def on_message(self, client, userdata, msg):
        """ Event callback trigger when mqtt subscription recieves a message """

        # If farm requesting new config info from fields
        if msg.topic == 'REFRESH_FIELDS':
            print('|','MQTT Request to resend config.')
            self.send_config()
            return

        # If message is one we are sending, skip
        if msg.topic not in self.mqtt_subscribers:
            return

        # Event callback trigger when mqtt client recieves a message
        print('')
        print('|','MQTT Message received ['+msg.topic+']')

        # Parse message to ROS
        t = msg.topic
        subs = [t['mqtt_namespace']+msg.topic for t in self.config['TOPICS'].values()]
        if [t for t in self.config if msg.topic.endswith(t)]:
            pass


def main(args=None):
    rclpy.init(args=args)

    FC = FarmConnector()
    rclpy.spin(FC)

    FC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

