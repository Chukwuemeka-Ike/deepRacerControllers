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
from _ServoCtrlMsg import ServoCtrlMsg # ctrl_pkg.msg
import numpy as np

# Create a laneDriver class to encapsulate the driving commands
class laneDriver:
    def __init__(self):
        self.servoCtrlMsg = ServoCtrlMsg()

    def driveForward(self):
        self.servoCtrlMsg.throttle = 0.

        return self.servoCtrlMsg

    def steerDeepRacer(self, angle):
        self.servoCtrlMsg.angle = angle

        return self.servoCtrlMsg

    def stopDriving(self):
        self.servoCtrlMsg.throttle = 0.0
        self.servoCtrlMsg.angle = 0.0

        return self.servoCtrlMsg
