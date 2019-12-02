#!/usr/bin/env python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import math
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
# ROS Image message
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import numpy as np
pic_resize = 400
# Instantiate CvBridge
bridge = CvBridge()
# given PATTERN
path = r'/home/jonathan/catkin_ws/src/image_detect/picture/'
name = ["Obama", "Avril", "Levi", "Bloom", "chinese", "No picture"] #keep no picture at least
pic = []
pic.append(cv2.imread(path+'obama.jpg'))
pic.append(cv2.imread(path+'avril.jpg'))
pic.append(cv2.imread(path+'levi.jpg'))
pic.append(cv2.imread(path+'bloom.jpg'))
pic.append(cv2.imread(path+'chinese.jpg'))

pic_len = len(pic)
# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()
# find the keypoints and descriptors with SIFT
des= [None] * pic_len
kp = [None] * pic_len
for i in range(0, pic_len-1):
    pic[i] = cv2.resize(pic[i], (pic_resize,pic_resize), interpolation = cv2.INTER_AREA)
    kp[i], des[i] = sift.detectAndCompute(pic[i],None)
# BFMatcher with default params
bf = cv2.BFMatcher()

def compare(i, cam_img): #i = id of given picture, cam_img = camera
    # find the keypoints and descriptors with SIFT
    try:
        kp_cam, des_cam = sift.detectAndCompute(cam_img,None)
        matches = bf.knnMatch(des[i],des_cam,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append([m])

         #cv.drawMatchesKnn expects list of lists as matches.
        img3 = cv2.drawMatchesKnn(pic[i],kp[i],cv2_img,kp_cam,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow('look', img3)    #real time camera
        cv2.waitKey(1)
        return len(good) #how similar
    except:
        return -1

def image_callback(msg):
    global val_sim, cv2_img
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "mono16")
        #cv2_img = cv2.Canny(cv2_img,100,200)
        max_id = 0
        max = compare(0, cv2_img)
        for i in range(0, pic_len-1):
            this = compare(i, cv2_img)
            if(this>max):
                max_id=i
                max=this
        if(max<20): #thresholding for no picture detected
            max_id=len(name)-1
        print(name[max_id], max)

    except CvBridgeError, e:
        print(e)

def main():
    rospy.init_node('image_detect')
    # Define your image topic
    image_topic = "/vrep/image"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
