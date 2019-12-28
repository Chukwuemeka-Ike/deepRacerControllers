#!/usr/bin/env python

# A node to first follow a line which will then be modified to follow a lane
# The DeepRacer will only drive when a lane is present to ease testing

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2, cv_bridge

class laneFollower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('cv_camera/image_rect_color', Image, self.image_callback)
        #self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([100, 20, 60])
        upper_yellow = np.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        cv2.imshow("window", mask )
        cv2.waitKey(3)

rospy.init_node('lane_follower')
follower = laneFollower()
rospy.spin()
