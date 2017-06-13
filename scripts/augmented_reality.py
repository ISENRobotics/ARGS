#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4*5,3), np.float32)
objp[:,:2] = np.mgrid[0:5,0:4].T.reshape(-1,2)

axis = np.float32([[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)


def draw(img, corners, imgpts):
    lineWidth = 2
    lineLenght = 6
    borderColor = (154,18,179)
    corner = tuple(corners[0].ravel())

    cv2.line(img, corner, tuple(imgpts[0].ravel()), borderColor, lineWidth)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), borderColor, lineWidth)

    corner_x = corner[0]
    corner_y = corner[1]
    ptX_x = imgpts[0][0][0]
    ptX_y = imgpts[0][0][1]
    ptY_x = imgpts[1][0][0]
    ptY_y = imgpts[1][0][1]

    vectX = ( ptX_x - corner_x , ptX_y - corner_y )
    vectX_away = ( lineLenght * vectX[0], lineLenght * vectX[1] )
    vectY = ( ptY_x - corner_x , ptY_y - corner_y )
    vectY_away = ( lineLenght * vectY[0], lineLenght * vectY[1] )

    cv2.line(img, corner, ( int( corner_x + vectX_away[0] ), int( corner_y + vectX_away[1] ) ), borderColor, lineWidth)

    cv2.line(img, corner, ( int( corner_x + vectY_away[0] ), int( corner_y + vectY_away[1] ) ), borderColor, lineWidth)

    cv2.line(
    img,
    ( int( corner_x + vectY_away[0] ), int( corner_y + vectY_away[1] ) ),
    ( int( corner_x + vectY_away[0] + vectX_away[0] ), int( corner_y + vectY_away[1] + vectX_away[1] ) ),
    borderColor,
    lineWidth)

    cv2.line(
    img,
    ( int( corner_x + vectX_away[0] ), int( corner_y + vectX_away[1] ) ),
    ( int( corner_x + vectY_away[0] + vectX_away[0] ), int( corner_y + vectY_away[1] + vectX_away[1] ) ),
    borderColor,
    lineWidth)

    #cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)



class image_feature:

    def __init__(self):

        self.initialized = 0;

        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/rgb/image_color/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/rgb/image_color/raw"


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # convert np image to grayscale
        gray = cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)

        # Detect pattern (chessboard)
        ret, corners = cv2.findChessboardCorners(gray, (5,4), None) 

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners)

            if (self.initialized == 0):
                self.initialized = 1;
                ret, self.camera_mtx, self.camera_dist, self.camera_rvecs, self.camera_tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

            # Find the rotation and translation vectors.
            self.camera_rvecs, self.camera_tvecs, inliers = cv2.solvePnPRansac(objp, corners, self.camera_mtx, self.camera_dist)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, self.camera_rvecs, self.camera_tvecs, self.camera_mtx, self.camera_dist)

            # Save
            self.previous_corners = corners
            self.previous_imgpts = imgpts

            draw(image_np,corners,imgpts)
        else:
            if (self.initialized == 1):
                draw(image_np,self.previous_corners,self.previous_imgpts)

        # Draw and display the corners
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(100)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
