#!/usr/bin/env python

# A node to first follow a line which will then be modified to follow a lane
# The DeepRacer will only drive when a lane is present to ease testing

import rospy
import numpy as np
from sensor_msgs.msg import Image
from _ServoCtrlMsg import ServoCtrlMsg # ctrl_pkg.msg
import cv2, cv_bridge
# import lane_driver as lDriver
import lane_finder as lFinder

# Create an overarching laneFollower class which encompasses the finder and
# driver classes from the other two scripts, and
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
    #
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        finder = lFinder.laneFinder(self.image)
        laneLines = lFinder.detectLane(finder)
        if len(laneLines) != 0 :
            lineImage = lFinder.displayLines(finder, laneLines)
            angle, headingImg = lFinder.computeSteeringAngle(finder, laneLines, lineImage)
            print(angle)
            # cv2.imshow("Lanes", headingImg)
            # cv2.waitKey(500)
            self.steerDeepRacer(angle)
        else:
            self.stopDriving()

    def steerDeepRacer(self, angle):
        self.servoCtrlMsg.throttle = 0.6
        self.servoCtrlMsg.angle = angle
        self.driveCommand.publish(self.servoCtrlMsg)

    def stopDriving(self):
        self.servoCtrlMsg.throttle = 0.0
        self.servoCtrlMsg.angle = 0.0
        self.driveCommand.publish(self.servoCtrlMsg)


rospy.init_node('lane_follower')
follower = laneFollower()
rospy.spin()
