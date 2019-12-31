#!/usr/bin/env python

import cv2, numpy as np

# find the webcam
capture = cv2.VideoCapture('vid1.avi')

# record video
while (capture.isOpened()):
    ret, frame = capture.read()
    cap = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow('frame', cap)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


capture.release()
cv2.destroyAllWindows()
