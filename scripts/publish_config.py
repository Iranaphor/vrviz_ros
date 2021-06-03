#!/usr/bin/env python
import sys
from typing import no_type_check_decorator
import rospy
import yaml
from pprint import pprint
from tf2_ros.buffer import Buffer

from vrviz_ros.msg import SystemConfig, Topic, UnityModifier, MqttParameters, FrameTransform

from tf2_ros.buffer import Buffer # github_src https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/buffer.py
from tf2_ros.transform_listener import TransformListener


class FrameTransformer(object):
    def format(self):
        return FrameTransform(self.parent_frame, self.child_frame, self.transforms)
    
    def __init__(self, parent_child, tf_buffer):
        self.parent_frame, self.child_frame = parent_child.split("parent: ")
        self.buffer = tf_buffer
        self.transforms = self.buffer.lookup_transform(self.child_frame, self.parent_frame, time=rospy.Time())
        rospy.logdebug_once(type(self.transforms))
        
        # time=0 gets the latest transform published


def get_frames():
    _tf_buffer = Buffer()#1
    _tf_listener = TransformListener(_tf_buffer)#2
    rospy.sleep(1)
    s = _tf_buffer.all_frames_as_yaml()#3
    # ft_list = [ FrameTransformer(pair.replace("'", ""), _tf_buffer).format() for pair in s.replace(": \n  ", "").split("\n") if "parent:" in pair]#4
    
    ft_list = []
    for pair in s.replace(": \n  ", "").replace("'","").split("\n"):
        if "parent:" in pair:
            ft_list.append(_tf_buffer.lookup_transform(pair.split("parent: ")[1], pair.split("parent: ")[0], rospy.Time(0)))
    return ft_list

def get_config_data(config_file):
    with open(config_file, "r") as f_handle:
        config_data = yaml.load(f_handle, Loader=yaml.FullLoader)
        return config_data


def validate_field(field_container, field_id, default="", required=False):
    if required and field_id not in field_container: sys.exit("Field {} is not present.".format(field_id))
    if field_id not in field_container: field_container[field_id] = default
    return field_container[field_id]


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit("Not enough parameters. Requires at least one parameter that is a path to a config yaml file.")
    if sys.argv[1] is None:
        sys.exit("No config file set!")
    if sys.argv[1] == "default":
        sys.argv[1] = "../config/default_config.yaml"


    config_data = get_config_data(sys.argv[1])
    if not config_data:
        sys.exit("Config file does not exist.")


    debug_level = rospy.get_param('vrviz_debug_level', '')
    if debug_level == 'debug':
        rospy.init_node('vrviz_config_publisher', anonymous=False, log_level=rospy.DEBUG) #TODO: simplify this conditional
    else:
        rospy.init_node('vrviz_config_publisher', anonymous=False)




    rospy.sleep(1)


    system_config_obj = SystemConfig()
    system_config_obj.topic_list = []
    for topic in config_data['topic_list']:
        print("\n\n\n-=-=-=-=-=-=-=-")
        topic['topic'] = validate_field(topic, 'topic', required=True)
        topic['msg_type'] = validate_field(topic, 'msg_type', required=True)
        
        topic['unity']= validate_field(topic, 'unity', required=True)
        topic['unity']['type'] = validate_field(topic['unity'], 'type', required=True)
        topic['unity']['target_frame'] = validate_field(topic['unity'], 'target_frame', required=False)
        topic['unity']['component'] = validate_field(topic['unity'], 'component', required=False)
        topic['unity']['modifier'] = validate_field(topic['unity'], 'modifier', required=True)
        topic['unity'] = UnityModifier(**topic['unity'])

        topic['mqtt'] = validate_field(topic, 'mqtt', default=dict().copy())
        topic['mqtt']['control_topic'] = validate_field(topic['mqtt'], 'control_topic', default="__dynamic_server");
        topic['mqtt']['mqtt_reference'] = validate_field(topic['mqtt'], 'mqtt_reference', default='vrviz'+topic['topic']);
        topic['mqtt']['frequency'] = validate_field(topic['mqtt'], 'frequency', default=1.0);
        topic['mqtt']['latched'] = validate_field(topic['mqtt'], 'latched', default=False);
        topic['mqtt']['qos'] = validate_field(topic['mqtt'], 'qos', default=2);
        topic['mqtt'] = MqttParameters(**topic['mqtt'])

        system_config_obj.topic_list.append(Topic(**topic))

        rospy.logdebug(system_config_obj)
        print("\n\n\n-=-=-=-=-=-=-=-")

    system_config_obj.frame_list = get_frames()


    pprint(system_config_obj)

    pub = rospy.Publisher('/vrviz/config', SystemConfig, latch=True, queue_size=5)
    pub.publish(system_config_obj)
    rospy.spin()
