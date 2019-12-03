#!/usr/bin/env python
import cv2
import numpy as np


def colorFiltering(img): #yellow filter
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #hsv.shape[1]
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    filtered = cv2.bitwise_and(img, img, mask = mask) #return filtered yellow picture
    (thres, bw) = cv2.threshold(cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY)
    return bw


cv2_img = cv2.imread('circle.png')

cv2.imshow('detected circles',colorFiltering(cv2_img))
cv2.waitKey(0)
