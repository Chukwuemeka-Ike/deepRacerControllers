#!/usr/bin/env python

# Code to drive the DeepRacer based on the lanes found using lane_classer.py

import lane_classer as lc
import cv2, numpy as np


# Simple function to compute the angle between the heading line and center line
def angleBtwPoints(x1, y1, x, y2):
    centerLine = -(y1-y2)
    # headingLine = np.sqrt(((x1-x)**2)+((y1-y2)**2))
    headingLine = (x1-x)
    # print(headingLine)
    # print(centerLine)
    angle = np.arctan2(headingLine,centerLine)
    # angle = np.arccos(((centerLine**2)+(headingLine**2)-(headingToCenter**2))/(2*centerLine*headingLine))
    return angle

# This function computes the steering angle
def computeSteeringAngle(laneLines):
    # Steering angle of:
    # 0 to 30 (0 to 0.9): turn left
    # 0 (0): straight
    # -30 to 0 (-0.9 to 0): turn right

    headingImg = np.zeros_like(lc.finder.image)
    height, width, _ = lc.finder.image.shape
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
    headingImg = cv2.addWeighted(lc.lineImage, 0.8, headingImg, 1, 1)
    angle = angleBtwPoints(xMid, yMid, x2, yEnd)*1.56
    cv2.imshow("Heading Image", headingImg)
    cv2.waitKey(1000)
    if angle > 0.9:
        angle = 0.9
    elif angle < -0.9:
        angle = -0.9

    return angle


angle = computeSteeringAngle(lc.laneLines)
