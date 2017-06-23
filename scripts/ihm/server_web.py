#!/usr/bin/env python

# Python libs
import sys, time

# Ros libraries
import roslib
import rospy
from geometry_msgs.msg import Twist
# Ros Messages
from sensor_msgs.msg import CompressedImage
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# HTTP libs
import BaseHTTPServer
import CGIHTTPServer

VERBOSE=False
init_serv=True

def _server(self):
    PORT = 8888
    server_address = ("", PORT)

    server = BaseHTTPServer.HTTPServer
    handler = CGIHTTPServer.CGIHTTPRequestHandler
    handler.cgi_directories = ["/"]
    print "Serveur actif sur le port :", PORT
    httpd = server(server_address, handler)
    httpd.serve_forever()


class server_web:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /usb_cam/image_raw/compressed"
        rospy.spin

    def callback(self, ros_data):
        '''Callback function of subscribed topic.'''
        if init_serv:
            _server(self)
            print "Server web start !"
            global init_serv
            init_serv=False

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # Draw and display the corners
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(5)



def main(args):
    '''Initializes and cleanup ros node'''

    ic = server_web()
    rospy.init_node('server_web', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
