#!/usr/bin/env python

import time
import rospy
from socket import socket, SOCK_DGRAM, AF_INET, SOL_SOCKET, SO_BROADCAST, gethostbyname, gethostname

if __name__ == "__main__":
    rospy.init_node('client_connection_finder', anonymous=True)

    mqtt_port = rospy.get_param("~mqtt_port", "7781")
    web_server_port = rospy.get_param("~web_server_port", "8080")
    PORT = 8608  # MUST be the same as VRViz.Pipeline.NMap.PORT
    # Identifer-ipv4-port
    data = "vrviz_ros-" + gethostbyname(gethostname()) + "-" + str(mqtt_port)+"-"+str(web_server_port)
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(("", 0))  # port to broadcast from
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    # b_data = bytes(data, "utf-8") # PY3
    rospy.sleep(5)

    while not rospy.is_shutdown():
        s.sendto(data, ("<broadcast>", PORT))
        rospy.logdebug("Sent data to broadcast, listening on port %d" % PORT)
        rospy.sleep(10)

        # rospy.logdebug("Status: %s" % status)