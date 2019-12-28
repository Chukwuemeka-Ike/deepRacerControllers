#!/usr/bin/env python

# Code to drive the DeepRacer based on the lanes found using lane_classer.py

import lane_classer as lc
import cv2, numpy as np

# This function takes in a lane finder object, and
def displayHeadingLine(finder, left_x2, right_x2, lineColor=(0, 0, 255), lineWidth=6):
    headingImg = np.zeros_like(finder.image)
    height, width, _ = finder.image.shape

    # Get the heading line from the steering angle
    # (x1, y1) always at bottom center of the screen
    # (x2, y2) is based on the angle and (x1, y1)

    # Steering angle of:
    # 0 to 30 (0 to 0.9): turn left
    # 0 (0): straight
    # -30 to 0 (-0.9 to 0): turn right
    # steeringAngleRadian = (steeringAngle)* np.pi/180

    x1 = int(width/2)
    y1 = height
    x2 = (left_x2+right_x2)/2
    # x2 = int(x1 - height/ 2 /np.tan(steeringAngleRadian))
    y2 = int(height/2)
    print(x2)
    cv2.line(headingImg, (x1,y1), (x2,y2), lineColor, lineWidth)
    headingImg = cv2.addWeighted(lc.lineImage, 0.8, headingImg, 1, 1)

    return headingImg


# Two detected lines
height, width, _ = lc.finder.image.shape
_, _, left_x2, _ = lc.laneLines[0][0]
_, _, right_x2, _ = lc.laneLines[1][0]


# Only one detected line
# x1, _, x2, _ = lc.laneLines[0][0]
# xOffset = x2 - x1
# yOffset = int(height/2)

# Calculate the steering angle
# angleToMidRad = np.arctan2(yOffset, xOffset) # angle # angle in radian to
#                                         # center vertical line
# angleToMidDeg = int(angleToMidRad*180/np.pi)
# steeringAngle = angleToMidDeg + 90

# if twoDetectedLines:
    headingImg = displayHeadingLine(lc.finder, left_x2, right_x2)
# else:
#    headingImg =
cv2.imshow("Heading Image", headingImg)
cv2.waitKey(600)
print(lc.laneLines)
print(yOffset)
print(xOffset)
print(steeringAngle)
