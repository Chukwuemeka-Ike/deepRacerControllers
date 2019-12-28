#!/usr/bin/env python

# A node to first follow a line which will then be modified to follow a lane
# The DeepRacer will only drive when a lane is present to ease testing

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2, cv_bridge
import lane_driver as ld
import lane_finder as lf

class laneFollower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('cv_camera/image_rect_color', Image, self.image_callback)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        lf.laneFinder(self.image)


rospy.init_node('lane_follower')
follower = laneFollower()
rospy.spin()
