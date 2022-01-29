#!/usr/bin/env python

# python 2
from SimpleHTTPServer import SimpleHTTPRequestHandler
from BaseHTTPServer import HTTPServer as BaseHTTPServer
import os
import rospy


class HTTPHandler(SimpleHTTPRequestHandler):
    """This handler uses server.base_path instead of always using os.getcwd()"""
    def translate_path(self, path):
        path = SimpleHTTPRequestHandler.translate_path(self, path)
        relpath = os.path.relpath(path, os.getcwd())
        fullpath = os.path.join(self.server.base_path, relpath)
        return fullpath


class HTTPServer(BaseHTTPServer):
    """The main server, you pass in base_path which is the path you want to serve requests from"""
    def __init__(self, base_path, server_address, RequestHandlerClass=HTTPHandler):
        self.base_path = base_path
        BaseHTTPServer.__init__(self, server_address, RequestHandlerClass)


if __name__ == '__main__':
    rospy.init_node('configuration_server', anonymous=True)
    web_server_port = rospy.get_param("~web_server_port", "8080")
    config_directory_file = rospy.get_param("~web_server_config", "../config/")
    web_dir = os.path.join(os.path.dirname(__file__), config_directory_file)
    http_daemon = HTTPServer(web_dir, ("", web_server_port))
    rospy.logdebug("Starting webserver on port %d" % web_server_port)
    while not rospy.is_shutdown():
        http_daemon.serve_forever()

# import rospy
# import BaseHTTPServer
# from SimpleHTTPServer import SimpleHTTPRequestHandler
#
#
# class customHandler(SimpleHTTPRequestHandler):
#     def do_GET(self):
#         temp = self.path
#         self.path = "configs/"+temp
#         return SimpleHTTPRequestHandler.do_GET(self)
#
#
# if __name__ == "__main__":
#     rospy.init_node('configuration_server', anonymous=True)
#     web_server_port = rospy.get_param("~web_server_port", "8080")
#     HandlerClass = SimpleHTTPRequestHandler
#     ServerClass = BaseHTTPServer.HTTPServer
#     Protocol = "HTTP/1.0"
#     server_address = ('127.0.0.1', web_server_port)
#
#     rospy.loginfo("Launching webserver on 127.0.0.1:%d" % web_server_port)
#     HandlerClass.protocol_version = Protocol
#     httpd = ServerClass(server_address, HandlerClass)
#     httpd.serve_forever()


    # mqtt_port = rospy.get_param("~mqtt_port", "7781")
    # PORT = 8608  # MUST be the same as VRViz.Pipeline.NMap.PORT
    # # Identifer-ipv4-port
    # data = "vrviz_ros-" + gethostbyname(gethostname()) + "-" + str(mqtt_port)
    # s = socket(AF_INET, SOCK_DGRAM)
    # s.bind(("", 0))  # port to broadcast from
    # s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    # # b_data = bytes(data, "utf-8") # PY3
    # rospy.sleep(5)
    #
    # while not rospy.is_shutdown():
    #     s.sendto(data, ("<broadcast>", PORT))
    #     rospy.logwarn("Sent data to broadcast, listening on port %d" % PORT)
    #     rospy.sleep(10)



        #rospy.logdebug("Status: %s" % status)