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
    def __init__(self, image):
        # self.bridge = cv_bridge.CvBridge()
        # self.image_sub = rospy.Subscriber('cv_camera/image_rect_color', Image,
        #                                             self.image_callback)
        self.image = image

    # def imageCallback(self, msg):
    #     self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #     laneLines = detectLane(self)
    #     lineImage = displayLines(self, laneLines)
    #     angle = computeSteeringAngle(laneLines, lineImage)
    #     print(angle)
    #     cv2.imshow("Lane Lines", lineImage)
    #     cv2.waitKey(1000)

    # Find the edges of the regions that correspond to the lane color
    def findEdges(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lowerBlue = np.array([60, 60, 60])
        upperBlue = np.array([150, 180, 90])
        mask = cv2.inRange(hsv, lowerBlue, upperBlue)
        self.edges = cv2.Canny(mask, 200, 400)
        return self.edges

    # This function helps the DeepRacer focus on the lower half of the image,
    # as this is the portion of immediate interest in steering the robot
    def immediateInterest(self):
        height, width = self.edges.shape
        mask = np.zeros_like(self.edges)

        # focus on the lower half
        polygon = np.array([[
            (0, height*1/2),
            (width, height*1/2),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        self.croppedEdges = cv2.bitwise_and(self.edges, mask)
        return self.croppedEdges

    # This function uses the Hough Line Transform in detecting line segments
    # from the computed edges from the above function.
    def detectLineSegments(self):
        rho = 0.2 # distance precision in pixels
        angle = np.pi/180 # angular precision in radians (1 degree)
        minThreshold = 20 # minimum length to be detected as a line
        self.lineSegments = cv2.HoughLinesP(self.croppedEdges, rho, angle, minThreshold,
                                np.array([]), minLineLength=15, maxLineGap=6)
        return self.lineSegments

    # helper
    def make_points(self, line):
        height, width, _ = self.image.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def avgSlopeIntercept(self):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then it's only right lane
        If all line slopes are > 0: then it's only left lane
        """
        laneLines = []
        if self.lineSegments is None:
            rospy.loginfo("No line segments detected")
            return laneLines

        height, width, _ = self.image.shape
        leftFit = []
        rightFit = []

        boundary = 2/5
        leftRegionBoundary = width*(1-boundary) # left lane line segment
                            # should be on left 2/3 of the screen
        rightRegionBoundary = width * boundary # right lane line segment
                            # should be on right 2/3 of the screen

        for lineSegment in self.lineSegments:
            for x1, y1, x2, y2 in lineSegment:
                if x1 == x2:
                    rospy.loginfo('No line segment detected')
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < leftRegionBoundary and x2 < leftRegionBoundary:
                        leftFit.append((slope, intercept))
                else:
                    if x1 > rightRegionBoundary and x2 > rightRegionBoundary:
                        rightFit.append((slope, intercept))
        leftFitAvg = np.average(leftFit, axis=0)
        if len(leftFit) > 0:
            laneLines.append(self.make_points(leftFitAvg))

        rightFitAvg = np.average(rightFit, axis=0)
        if len(rightFit) > 0:
            laneLines.append(self.make_points(rightFitAvg))
        rospy.loginfo("Lane lines: %s" % laneLines)
        return laneLines


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
def computeSteeringAngle(finder, laneLines, lineImage):
    # Steering angle of:
    # 0 to 30 (0 to 0.9): turn left
    # 0 (0): straight
    # -30 to 0 (-0.9 to 0): turn right

    headingImg = np.zeros_like(finder.image)
    height, width, _ = finder.image.shape
    xMid = int(width/2)
    yEnd = height
    yMid = int(height/2)

    if len(laneLines) > 0:
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
    else:
        angle = 0.0
    return angle, headingImg
