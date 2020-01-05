#!/usr/bin/env python

# A node to first follow a line which will then be modified to follow a lane
# The DeepRacer will only drive when a lane is present to ease testing

import rospy
import numpy as np
from sensor_msgs.msg import Image
from _ServoCtrlMsg import ServoCtrlMsg # ctrl_pkg.msg
import cv2, cv_bridge
import lane_finder as lFinder

# Set the forward throttle value to be 0.5 which is about 0.318 m/s
forwardThrottle = 0.5

# Create a laneFollower class which encompasses the finder
# functionalities and uses them in sending ServoCtrlMsg messages to drive
# the DeepRacer
class laneFollower:
    #
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('cv_camera/image_rect_color',
                                                Image, self.image_callback)
        self.driveCommand = rospy.Publisher('manual_drive', ServoCtrlMsg,
                                                        queue_size=30)
        self.servoCtrlMsg = ServoCtrlMsg()

    #
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        finder = lFinder.laneFinder(self.image)
        laneLines = lFinder.detectLane(finder)
        cv2.waitKey(1000)
        if len(laneLines) != 0 :
            lineImage = lFinder.displayLines(finder, laneLines)
            angle, headingImg = lFinder.computeSteeringAngle(finder, laneLines, lineImage)
            cv2.imshow("Lanes", headingImg)
            self.steerDeepRacer(angle)
        else:
            self.stopDriving()

    def steerDeepRacer(self, angle):
        self.servoCtrlMsg.throttle = forwardThrottle
        self.servoCtrlMsg.angle = angle
        self.driveCommand.publish(self.servoCtrlMsg)

    def stopDriving(self):
        self.servoCtrlMsg.throttle = 0.0
        self.servoCtrlMsg.angle = 0.0
        self.driveCommand.publish(self.servoCtrlMsg)


rospy.init_node('lane_follower')
follower = laneFollower()
rospy.spin()
