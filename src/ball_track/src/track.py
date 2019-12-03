#!/usr/bin/env python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
# Instantiate CvBridge
bridge = CvBridge()
def colorFiltering(img): #yellow filter
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #hsv.shape[1]
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    filtered = cv2.bitwise_and(img, img, mask = mask) #return filtered yellow picture
    (thres, bw) = cv2.threshold(cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY)
    return bw

def image_callback(msg):
    global val_sim, cv2_img
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = cv2.blur(cv2_img, (10, 10))
        cv2.imshow('detected',colorFiltering(cv2_img))
        cv2.waitKey(1)
    except CvBridgeError, e:
        print(e)

def main():
    rospy.init_node('ball_track')
    # Define your image topic
    image_topic = "/vrep/image"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
