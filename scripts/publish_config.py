#!/usr/bin/env python
import sys
from typing import no_type_check_decorator
import rospy
import yaml
from pprint import pprint
from tf2_ros.buffer import Buffer

from vrviz_ros.msg import SystemConfig, Topic, UnityModifier, MqttParameters
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

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

    debug_level = sys.argv[1] if (len(sys.argv) > 1) else None

    if debug_level == 'debug':
        rospy.init_node('vrviz_config_publisher', anonymous=False, log_level=rospy.DEBUG) #TODO: simplify this conditional
    else:
        rospy.init_node('vrviz_config_publisher', anonymous=False)


    print(rospy.get_param("~topic_config", "1"))
    print(rospy.get_param("~mesh_config", "2"))

    config_data = get_config_data( rospy.get_param("~topic_config", ""))
    mesh_config_data = get_config_data( rospy.get_param("~mesh_config", ""))



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



    system_config_obj.mesh_list = []
    for mesh in mesh_config_data['mesh_list']:
        marker_obj = Marker()

        mesh['frame_id'] = validate_field(mesh, 'frame_id', required=True)
        mesh['ns'] = validate_field(mesh, 'shape_name', default="model_"+mesh['frame_id'].replace("_link",""))
        mesh['type'] = validate_field(mesh, 'type', required=True)
        marker_obj.header.frame_id = mesh['frame_id']
        marker_obj.ns = mesh['shape_name']
        marker_obj.type = getattr(Marker(), mesh['type'])

        mesh['details']= validate_field(mesh, 'details', required=True)

        mesh['details']['position'] = validate_field(mesh['details'], 'position', default=dict())
        mesh['details']['position']['x'] = validate_field(mesh['details']['position'], 'x', default=0)
        mesh['details']['position']['y'] = validate_field(mesh['details']['position'], 'y', default=0)
        mesh['details']['position']['z'] = validate_field(mesh['details']['position'], 'z', default=0)
        marker_obj.pose.position = Point(**mesh['details']['position'])

        mesh['details']['orientation'] = validate_field(mesh['details'], 'orientation', default=dict())
        mesh['details']['orientation']['x'] = validate_field(mesh['details']['orientation'], 'x', default=0)
        mesh['details']['orientation']['y'] = validate_field(mesh['details']['orientation'], 'y', default=0)
        mesh['details']['orientation']['z'] = validate_field(mesh['details']['orientation'], 'z', default=0)
        mesh['details']['orientation']['w'] = validate_field(mesh['details']['orientation'], 'w', default=0)
        marker_obj.pose.orientation = Quaternion(**mesh['details']['orientation']) #TODO: this doesnt work yet
        
        mesh['details']['scale'] = validate_field(mesh['details'], 'scale', default=dict())
        mesh['details']['scale']['x'] = validate_field(mesh['details']['scale'], 'x', default=0)
        mesh['details']['scale']['y'] = validate_field(mesh['details']['scale'], 'y', default=0)
        mesh['details']['scale']['z'] = validate_field(mesh['details']['scale'], 'z', default=0)
        marker_obj.scale = Vector3(**mesh['details']['scale'])

        mesh['details']['color'] = validate_field(mesh['details'], 'color', default=dict())
        mesh['details']['color']['r'] = validate_field(mesh['details']['color'], 'r', default=0)
        mesh['details']['color']['b'] = validate_field(mesh['details']['color'], 'b', default=0)
        mesh['details']['color']['g'] = validate_field(mesh['details']['color'], 'g', default=0)
        mesh['details']['color']['a'] = validate_field(mesh['details']['color'], 'a', default=0)
        marker_obj.color = ColorRGBA(**mesh['details']['color'])

        system_config_obj.mesh_list.append(marker_obj)


    pprint(system_config_obj)

    pub = rospy.Publisher('/vrviz/config', SystemConfig, queue_size=5)

    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            rospy.sleep(5.0)
            pub.publish(system_config_obj)
            break
            
    rospy.spin()
