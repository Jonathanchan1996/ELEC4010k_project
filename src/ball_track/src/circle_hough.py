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

img = cv2.imread('circle.png')
img = colorFiltering(img)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,2,50,param1=50,param2=30,minRadius=0,maxRadius=100)
if circles is None:
    print("No circle is detected")
    
circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

print("center:(",circles[0][0][0],",",circles[0][0][1],")")
print("radius:",circles[0][0][2])

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
