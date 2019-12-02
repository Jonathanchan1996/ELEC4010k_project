#!/usr/bin/env python
import cv2
path = r'/home/jonathan/catkin_ws/src/image_detect/picture/obama.jpg'
image = cv2.imread(path)
if image !=None:
    cv2.imshow('image', image)
    cv2.waitKey(0)
else:
    print "No image"
