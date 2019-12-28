#!/usr/bin/env python

# This script allows the DeepRacer find and display the visible lane lines
# in its field of view.
#
#
#

# Import the necessary modules to allow the script access and manipulate
# images from the onboard camera
import cv2, cv_bridge
import rospy
from sensor_msgs.msg import Image
import numpy as np

# Lane finder class
class laneFinder:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('cv_camera/image_rect_color', Image,
                                                    self.image_callback)
        self.image = []

    def imageCallback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        laneLines = detectLane(self)
        lineImage = displayLines(self, laneLines)
        angle = computeSteeringAngle(laneLines, lineImage)
        print(angle)
        cv2.imshow("Lane Lines", lineImage)
        cv2.waitKey(1000)


def detectLane(finder):
    finder.findEdges()
    finder.immediateInterest()
    finder.detectLineSegments()
    laneLines = finder.avgSlopeIntercept()
    return laneLines

def displayLines(finder, lines, lineColor=(255, 155, 0), lineWidth=4):
    lineImage = np.zeros_like(finder.image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(lineImage, (x1, y1), (x2, y2), lineColor, lineWidth)
    lineImage = cv2.addWeighted(finder.image, 0.8, lineImage, 1, 1)
    return lineImage

# Simple function to compute the angle between the heading line and center line
def angleBtwPoints(x1, y1, x, y2):
    centerLine = -(y1-y2)
    headingLine = (x1-x)
    angle = np.arctan2(headingLine,centerLine)

    return angle

# This function computes the steering angle
def computeSteeringAngle(laneLines, lineImage):
    # Steering angle of:
    # 0 to 30 (0 to 0.9): turn left
    # 0 (0): straight
    # -30 to 0 (-0.9 to 0): turn right

    headingImg = np.zeros_like(finder.image)
    height, width, _ = finder.image.shape
    xMid = int(width/2)
    yEnd = height
    yMid = int(height/2)

    # Only one detected line
    if len(laneLines) == 1:
        x1, y1, x2, y2 = laneLines[0][0]
        fit = np.polyfit((x1, x2), (y1, y2), 1)
        slope = fit[0]
        x2 = int((-yMid/slope)+xMid)

    # Two detected lines
    elif len(laneLines) == 2:
        _, _, left_x2, _ = laneLines[0][0]
        _, _, right_x2, _ = laneLines[1][0]
        x2 = (left_x2+right_x2)/2

    cv2.line(headingImg, (xMid,yEnd), (x2,yMid), (0,0,255), 10)
    headingImg = cv2.addWeighted(lineImage, 0.8, headingImg, 1, 1)
    angle = angleBtwPoints(xMid, yMid, x2, yEnd)*1.56
    if angle > 0.9:
        angle = 0.9
    elif angle < -0.9:
        angle = -0.9

    return angle
