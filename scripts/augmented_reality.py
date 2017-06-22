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
from collections import deque
from math import sqrt

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import *
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# config
chessWidth = 4
chessHeight = 5
scaleFactor = 2.05
crossWidth = 3
colorC = (242, 38, 19)
colorR = (30, 130, 76)
borderColor = (154,18,179)
img_treatment_freq = 1000000000 # 1 s
mean_size = 6

inputImagesTopic = "/usb_cam/image_raw/compressed"
ouputImagesTopic = "/augmented_reality_output/image_raw/compressed"
inputActionTopic = "/Case_Jeu"

showWindow = False
VERBOSE = False

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessWidth*chessHeight,3), np.float32)
objp[:,:2] = np.mgrid[0:chessHeight,0:chessWidth].T.reshape(-1,2)

# Unity axis, based on the chess
axis = np.float32( [[0,0,0], [3,0,0], [3,3,0], [0,3,0]] )


def _generatePlateCorners( self, imgpts ):
    imgpts = np.float32(imgpts).reshape(-1,2)

    # Get the intersection point between 01 and 23
    intersection0123 = self.intersection0123
    # Get the intersection point between 12 and 30
    intersection1230 = self.intersection1230

    corners = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    corners[0] = imgpts[0]

    dist01_x = scaleFactor * ( imgpts[1][0] - imgpts[0][0] )
    dist01_y = scaleFactor * ( imgpts[1][1] - imgpts[0][1] )
    corners[1] = ( int( corners[0][0] + dist01_x ), int( corners[0][1] + dist01_y ) )

    dist04_x = scaleFactor * ( imgpts[3][0] - imgpts[0][0] )
    dist04_y = scaleFactor * ( imgpts[3][1] - imgpts[0][1] )
    corners[4] = ( int( corners[0][0] + dist04_x ), int( corners[0][1] + dist04_y ) )

    corners[5] = _getIntersectionPoint( [corners[1], intersection1230], [corners[4], intersection0123] )
    dist15_x = int( corners[5][0] - corners[1][0] )
    dist15_y = int( corners[5][1] - corners[1][1] )
    dist45_x = int( corners[5][0] - corners[4][0] )
    dist45_y = int( corners[5][1] - corners[4][1] )

    long15 = sqrt( dist15_x * dist15_x + dist15_y * dist15_y )
    long45 = sqrt( dist45_x * dist45_x + dist45_y * dist45_y )
    long01 = sqrt( dist01_x * dist01_x + dist01_y * dist01_y )
    long04 = sqrt( dist04_x * dist04_x + dist04_y * dist04_y )
    thalesRatio1504 = long15 / long04
    thalesRatio4501 = long45 / long01

    dist12_x = thalesRatio1504 * dist01_x
    dist12_y = thalesRatio1504 * dist01_y
    corners[2] = ( int( corners[1][0] + dist12_x ), int( corners[1][1] + dist12_y ) )
    dist23_x = thalesRatio1504 * dist12_x
    dist23_y = thalesRatio1504 * dist12_y
    corners[3] = ( int( corners[2][0] + dist23_x ), int( corners[2][1] + dist23_y ) )

    dist48_x = thalesRatio4501 * dist04_x
    dist48_y = thalesRatio4501 * dist04_y
    corners[8] = ( int( corners[4][0] + dist48_x ), int( corners[4][1] + dist48_y ) )
    dist812_x = thalesRatio4501 * dist48_x
    dist812_y = thalesRatio4501 * dist48_y
    corners[12] = ( int( corners[8][0] + dist812_x ), int( corners[8][1] + dist812_y ) )

    corners[13] = _getIntersectionPoint( [corners[1], intersection1230], [corners[12], intersection0123] )
    corners[14] = _getIntersectionPoint( [corners[2], intersection1230], [corners[12], intersection0123] )
    corners[15] = _getIntersectionPoint( [corners[3], intersection1230], [corners[12], intersection0123] )

    corners[9]  = _getIntersectionPoint( [corners[1], intersection1230], [corners[8], intersection0123] )
    corners[10] = _getIntersectionPoint( [corners[2], intersection1230], [corners[8], intersection0123] )
    corners[11] = _getIntersectionPoint( [corners[3], intersection1230], [corners[8], intersection0123] )

    corners[6] = _getIntersectionPoint( [corners[2], intersection1230], [corners[4], intersection0123] )
    corners[7] = _getIntersectionPoint( [corners[3], intersection1230], [corners[4], intersection0123] )

    return corners


def _draw(img, corners):
    lineWidth = 2

    cv2.line(img, tuple(corners[0]), ( int( corners[12][0] ), int( corners[12][1] ) ) , borderColor, lineWidth)
    cv2.line(img, tuple(corners[1]), ( int( corners[13][0] ), int( corners[13][1] ) ) , borderColor, lineWidth)
    cv2.line(img, tuple(corners[2]), ( int( corners[14][0] ), int( corners[14][1] ) ) , borderColor, lineWidth)
    cv2.line(img, tuple(corners[3]), ( int( corners[15][0] ), int( corners[15][1] ) ) , borderColor, lineWidth)

    cv2.line(img, tuple(corners[0]), ( int( corners[3][0] ), int( corners[3][1] ) ) , borderColor, lineWidth)
    cv2.line(img, tuple(corners[4]), ( int( corners[7][0] ), int( corners[7][1] ) ) , borderColor, lineWidth)
    cv2.line(img, tuple(corners[8]), ( int( corners[11][0] ), int( corners[11][1] ) ) , borderColor, lineWidth)
    cv2.line(img, tuple(corners[12]), ( int( corners[15][0] ), int( corners[15][1] ) ) , borderColor, lineWidth)


def _mean_lines(self, imgpts):
    imgpts = np.float32(imgpts).reshape(-1,2)

    # Get the intersection point between 01 and 23
    intersection0123 = _getIntersectionPoint( [imgpts[0],imgpts[1]], [imgpts[2],imgpts[3]] )
    # Get the intersection point between 12 and 30
    intersection1230 = _getIntersectionPoint( [imgpts[1],imgpts[2]], [imgpts[3],imgpts[0]] )

    self.intersections0123.append( intersection0123 )
    if( len(self.intersections0123) > mean_size ):
        self.intersections0123.popleft()
    self.intersections1230.append( intersection1230 )
    if( len(self.intersections1230) > mean_size ):
        self.intersections1230.popleft()

    # Mean intersections0123
    mean_0123_x = 0
    mean_0123_y = 0
    for pt in self.intersections0123:
        mean_0123_x = mean_0123_x + pt[0]
        mean_0123_y = mean_0123_y + pt[1]
    mean_0123_x = mean_0123_x / len(self.intersections0123)
    mean_0123_y = mean_0123_y / len(self.intersections0123)
    self.intersection0123 = [ mean_0123_x, mean_0123_y ]

    # Mean intersections1230
    mean_1230_x = 0
    mean_1230_y = 0
    for pt in self.intersections1230:
        mean_1230_x = mean_1230_x + pt[0]
        mean_1230_y = mean_1230_y + pt[1]
    mean_1230_x = mean_1230_x / len(self.intersections1230)
    mean_1230_y = mean_1230_y / len(self.intersections1230)
    self.intersection1230 = [ mean_1230_x, mean_1230_y ]

    # Norme of unity vectors
    self.normes01.append( imgpts[1][0] - imgpts[0][0] )
    if( len(self.normes01) > mean_size ):
        self.normes01.popleft()
    self.normes03.append( imgpts[3][0] - imgpts[0][0] )
    if( len(self.normes03) > mean_size ):
        self.normes03.popleft()

    # Mean normes01
    norme01 = 0
    for pt in self.normes01:
        norme01 = norme01 + pt
    norme01 = norme01 / len(self.normes01)
    # Mean normes03
    norme03 = 0
    for pt in self.normes03:
        norme03 = norme03 + pt
    norme03 = norme03 / len(self.normes03)

    # Move all imgpts to be in the origin - intersection lines

    # y = ax+b
    # a = (y0-y1)/(x0-x1)
    # b = y0 - a*x0
    dirCoef_01 = ( imgpts[0][1] - imgpts[1][1] ) / ( imgpts[0][0] - imgpts[1][0] )
    origin_01 = imgpts[0][1] - ( dirCoef_01 * imgpts[0][0] )
    imgpts[1][0] = imgpts[0][0] + norme01
    imgpts[1][1] = ( dirCoef_01 * imgpts[1][0] ) + origin_01

    # y = ax+b
    # a = (y0-y1)/(x0-x1)
    # b = y0 - a*x0
    dirCoef_03 = ( imgpts[0][1] - imgpts[3][1] ) / ( imgpts[0][0] - imgpts[3][0] )
    origin_03 = imgpts[0][1] - ( dirCoef_03 * imgpts[0][0] )
    imgpts[3][0] = imgpts[0][0] + norme03
    imgpts[3][1] = ( dirCoef_03 * imgpts[3][0] ) + origin_03

    return imgpts


def _mean_origin(self, imgpts):
    imgpts = np.float32(imgpts).reshape(-1,2)

    # origin point
    self.origins.append([ imgpts[0][0], imgpts[0][1] ])
    if( len(self.origins) > mean_size ):
        self.origins.popleft()

    # Mean origin
    meanx = 0
    meany = 0
    for pt in self.origins:
        meanx = meanx + pt[0]
        meany = meany + pt[1]
    diffx = ( meanx / len(self.origins) ) - imgpts[0][0]
    diffy = ( meany / len(self.origins) ) - imgpts[0][1]

    # Move all points to have same origin
    imgpts[0][0] = imgpts[0][0] + diffx
    imgpts[1][0] = imgpts[1][0] + diffx
    imgpts[2][0] = imgpts[2][0] + diffx
    imgpts[3][0] = imgpts[3][0] + diffx
    imgpts[0][1] = imgpts[0][1] + diffy
    imgpts[1][1] = imgpts[1][1] + diffy
    imgpts[2][1] = imgpts[2][1] + diffy
    imgpts[3][1] = imgpts[3][1] + diffy

    return imgpts


def _getIntersectionPoint( seg1, seg2 ):
    # y = ax+b
    # a = (y0-y1)/(x0-x1)
    dirCoef_segment_1 = ( seg1[0][1] - seg1[1][1] ) / ( seg1[0][0] - seg1[1][0] )
    dirCoef_segment_2 = ( seg2[0][1] - seg2[1][1] ) / ( seg2[0][0] - seg2[1][0] )
    # b = y0 - a*x0
    origin_segment_1 = seg1[0][1] - ( dirCoef_segment_1 * seg1[0][0] )
    origin_segment_2 = seg2[0][1] - ( dirCoef_segment_2 * seg2[0][0] )

    # x of intersection = (b2-b1)/(a1-b2)
    intersection_x = ( origin_segment_2 - origin_segment_1 ) / ( dirCoef_segment_1 - dirCoef_segment_2 )
    # y corresponding
    intersection_y = ( dirCoef_segment_1 * intersection_x ) + origin_segment_1
    return [ intersection_x, intersection_y ]


def _draw_action_played(image_np, self):
    i = 0
    for player in self.plate:
        if player:
            _play(player, i, image_np, self.previous_plateCorners)
        i = i + 1


def _play(player, case, img, imgpts):
    # Color for players
    if( player == 1 ):
        color = colorC
    else:
        color = colorR
    # Draw cross
    imgpts = np.float32(imgpts).reshape(-1,2)
    cv2.line(img, tuple(imgpts[case]), tuple(imgpts[case+5]), color, crossWidth)
    cv2.line(img, tuple(imgpts[case+4]), tuple(imgpts[case+1]), color, crossWidth)


class augmented_reality:

    def __init__(self):
        self.initialized = 0
        self.last_treatment = 0
        self.origins = deque()
        self.intersections0123 = deque()
        self.normes01 = deque()
        self.intersections1230 = deque()
        self.normes03 = deque()
        self.plate = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        # Initialize ros publishers and ros subscribers

        # topic where we publish
        self.image_pub = rospy.Publisher(ouputImagesTopic, CompressedImage, queue_size = 1)

        # subscribed camera Topic
        self.subscriber = rospy.Subscriber(inputImagesTopic, CompressedImage, self.callback, queue_size = 1)

        # subscribed action topic
        self.action_subscriber = rospy.Subscriber(inputActionTopic, Int8MultiArray, self.action_callback, queue_size = 1)

        if VERBOSE :
            print "subscribed to %s" % (inputImagesTopic)
            print "subscribed to %s" % (inputActionTopic)


    # Callback function of subscribed action topic
    def action_callback(self, ros_data):
        print "received a spoiler of the game in action_callback"
        player = 1
        for case_nb in ros_data.data:
            # Received case numbers between 0 and 8
            # Transform it for corners
            if( case_nb >= 3 and case_nb <= 5 ):
                case_nb = case_nb + 1
            elif( case_nb >= 6 and case_nb <= 8 ):
                case_nb = case_nb + 2
            # Write in plate
            self.plate[case_nb] = player
            # switch player
            if( player == 1 ):
                player = 2
            else:
                player = 1


    # Callback function of subscribed camera topic
    def callback(self, ros_data):
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # Treat only image after img_treatment_freq (in ns) passed
        now = rospy.Time.now()
        now_ns = 1000000000 * now.secs + now.nsecs # time in ns
        if( now_ns - self.last_treatment >= img_treatment_freq ):
            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.

            # convert np image to grayscale
            gray = cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)

            # Detect pattern (chessboard)
            ret, corners = cv2.findChessboardCorners(gray, (chessHeight,chessWidth), None) 

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

                imgpts = _mean_origin(self, imgpts)
                imgpts = _mean_lines(self, imgpts)
                plateCorners = _generatePlateCorners(self, imgpts)
                self.previous_plateCorners = plateCorners

            # Save last treatment
            self.last_treatment = now_ns

        if (self.initialized == 1):
            # Draw plate lines
            _draw(image_np,self.previous_plateCorners)
            # Draw plate players cross
            _draw_action_played(image_np,self)


        # Display image
        if showWindow:
            cv2.imshow('cv_img', image_np)
            cv2.waitKey(5)

        # Create CompressedImage
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)


def main(args):
    # Initializes and cleanup ros node
    ar = augmented_reality()
    rospy.init_node('args_ar', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
