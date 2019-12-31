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

# find the webcam
capture = cv2.VideoCapture('vid2.avi')

# record video
while (capture.isOpened()):
    ret, frame = capture.read()

    #
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerYellow = np.array([10, 100, 50])
    upperYellow = np.array([40, 110, 120])
    mask = cv2.inRange(frame, lowerYellow, upperYellow)
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


    # # avgSlopeIntercept
    # rho = 0.2 # distance precision in pixels
    # angle = np.pi/180 # angular precision in radians (1 degree)
    # minThreshold = 20 # minimum length to be detected as a line
    # lineSegments = cv2.HoughLinesP(croppedEdges, rho, angle, minThreshold,
    #                         np.array([]), minLineLength=15, maxLineGap=6)
    #
    # laneLines = []
    #
    # leftFit = []
    # rightFit = []
    #
    # boundary = 2/5
    # leftRegionBoundary = width*(1-boundary) # left lane line segment
    #                     # should be on left 2/3 of the screen
    # rightRegionBoundary = width * boundary # right lane line segment
    #                     # should be on right 2/3 of the screen
    #
    # for lineSegment in lineSegments:
    #     for x1, y1, x2, y2 in lineSegment:
    #         if x1 == x2:
    #             continue
    #         fit = np.polyfit((x1, x2), (y1, y2), 1)
    #         slope = fit[0]
    #         intercept = fit[1]
    #         if slope < 0:
    #             if x1 < leftRegionBoundary and x2 < leftRegionBoundary:
    #                 leftFit.append((slope, intercept))
    #         else:
    #             if x1 > rightRegionBoundary and x2 > rightRegionBoundary:
    #                 rightFit.append((slope, intercept))
    # leftFitAvg = np.average(leftFit, axis=0)
    # if len(leftFit) > 0:
    #     laneLines.append(make_points(frame, leftFitAvg))
    #
    # rightFitAvg = np.average(rightFit, axis=0)
    # if len(rightFit) > 0:
    #     laneLines.append(make_points(frame, rightFitAvg))
    #
    #
    # # displayLines
    # lineImage = np.zeros_like(frame)
    # if laneLines is not None:
    #     for line in laneLines:
    #         for x1, y1, x2, y2 in line:
    #             cv2.line(lineImage, (x1, y1), (x2, y2), (255,155,0), 4)
    # lineImage = cv2.addWeighted(frame, 0.8, lineImage, 1, 1)

    cv2.imshow('frame', edges)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break


capture.release()
cv2.destroyAllWindows()
