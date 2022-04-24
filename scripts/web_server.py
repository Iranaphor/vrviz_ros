#!/usr/bin/env python

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
