#!/usr/bin/env python

import time
import rospy
from socket import socket, SOCK_DGRAM, AF_INET, SOL_SOCKET, SO_BROADCAST, gethostbyname, gethostname

if __name__ == "__main__":
    rospy.init_node('client_connection_finder', anonymous=True)

    # Collect port details from parameter server
    mqtt_port = rospy.get_param("~mqtt_port", "7781")
    web_server_port = rospy.get_param("~web_server_port", "8080")
    # conf_fn = rospy.get_param("~vrviz_config_file", "example.yaml")

    # Define the LAN port, this being the port VRViz is hard-coded to listen for information on
    # If this needs to be modified, ensure it is upated @ VRViz.Pipeline.LAN_PORT
    LAN_PORT = 8608

    # Identifier-ipv4-mqtt_port-web_server_port
    data = "vrviz_ros-" + gethostbyname(gethostname()) + "-" + str(mqtt_port)+"-"+str(web_server_port)
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(("", 0))  # Internal port to broadcast from
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    rospy.sleep(5)

    def publish(event):
        s.sendto(data, ("<broadcast>", LAN_PORT))
        rospy.logdebug("Sent data to broadcast, listening on port %d" % LAN_PORT)
    rospy.Timer(rospy.Duration(5), publish)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     s.sendto(data, ("<broadcast>", LAN_PORT))
    #     rospy.logdebug("Sent data to broadcast, listening on port %d" % LAN_PORT)
    #     rospy.sleep(10)
