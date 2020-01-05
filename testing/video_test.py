#!/usr/bin/env python

import cv2, numpy as np

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def angleBtwPoints(x1, y1, x, y2):
    centerLine = -(y1-y2)
    # headingLine = np.sqrt(((x1-x)**2)+((y1-y2)**2))
    headingLine = (x1-x)
    # print(headingLine)
    # print(centerLine)
    angle = np.arctan2(headingLine,centerLine)
    # angle = np.arccos(((centerLine**2)+(headingLine**2)-(headingToCenter**2))/(2*centerLine*headingLine))
    return angle


def computeSteeringAngle(frame, laneLines, lineImage):
    # Steering angle of:
    # 0 to 30 (0 to 0.9): turn left
    # 0 (0): straight
    # -30 to 0 (-0.9 to 0): turn right

    headingImg = np.zeros_like(frame)
    height, width, _ = frame.shape
    xMid = int(width/2)
    yEnd = height
    yMid = int(height/2)
    x2 = 0
    angle = 0

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

    if x2 != 0:
        cv2.line(headingImg, (xMid,yEnd), (x2,yMid), (0,0,255), 10)
        headingImg = cv2.addWeighted(lineImage, 0.8, headingImg, 1, 1)
        angle = angleBtwPoints(xMid, yMid, x2, yEnd)*1.56
        # cv2.imshow("Heading Image", headingImg)
        cv2.waitKey(1000)
        if angle > 0.9:
            angle = 0.9
        elif angle < -0.9:
            angle = -0.9

        cv2.imshow('Video Test' , headingImg)
    return angle


# find the webcam
capture = cv2.VideoCapture('./blueTape/vid1.avi')

# record video
while (capture.isOpened()):
    ret, frame = capture.read()

    #
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerBlue = np.array([60, 60, 60])
    upperBlue = np.array([150, 180, 90])
    mask = cv2.inRange(hsv, lowerBlue, upperBlue)
    edges = cv2.Canny(mask, 100, 200)

    #
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # immediateInterest
    polygon = np.array([[
        (0, height*1/2),
        (width, height*1/2),
        (width, height),
        (0, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    croppedEdges = cv2.bitwise_and(edges, mask)


    # avgSlopeIntercept
    rho = 4 # distance precision in pixels
    angle = 5*np.pi/180 # angular precision in radians (1 degree)
    minThreshold = 25 # minimum length to be detected as a line
    lineSegments = cv2.HoughLinesP(croppedEdges, rho, angle, minThreshold,
                            np.array([]), minLineLength=20, maxLineGap=10)
    # print(lineSegments)
    laneLines = []

    leftFit = []
    rightFit = []

    boundary = 2/5
    leftRegionBoundary = width*(1-boundary) # left lane line segment
                        # should be on left 2/3 of the screen
    rightRegionBoundary = width * boundary # right lane line segment
                        # should be on right 2/3 of the screen
    if lineSegments is not None:
        for lineSegment in lineSegments:
            # print(lineSegment)
            for x1, y1, x2, y2 in lineSegment:
                if x1 == x2:
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
        laneLines.append(make_points(frame, leftFitAvg))

    rightFitAvg = np.average(rightFit, axis=0)
    if len(rightFit) > 0:
        laneLines.append(make_points(frame, rightFitAvg))


    # displayLines
    lineImage = np.zeros_like(frame)
    if laneLines is not None:
        for line in laneLines:
            for x1, y1, x2, y2 in line:
                cv2.line(lineImage, (x1, y1), (x2, y2), (255,155,0), 4)
    lineImage = cv2.addWeighted(frame, 0.8, lineImage, 1, 1)

    # cv2.imshow('Video Test', lineImage)
    angle = computeSteeringAngle(frame, laneLines, lineImage)
    print(angle)

    if cv2.waitKey(3) & 0xFF == ord('q'):
        break


capture.release()
cv2.destroyAllWindows()
