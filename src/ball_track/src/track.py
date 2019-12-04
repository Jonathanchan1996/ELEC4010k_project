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
import sys
# Instantiate CvBridge
bridge = CvBridge()
#Kp, Ki, Kd, sp, I_accum, D_lastError, I_accum_max, max_limit, Past_output
lin_pid = np.array([1.0/256, 0.00/256, 0.01/256, 180, 0.0, 0.0, 1.,1., 0.0])
ang_pid = np.array([5.0/256, 0.01/256, 0.05/256, 256, 0.0, 0.0, 1.,2.5, 0.0])

def pid(PID, y):
    e=(y-PID[3])
    output = float(0)
    output = (PID[0]*e) + (PID[1]*PID[4]) + (PID[2]*(e-PID[5]))
    PID[4]=PID[4]+e
    if(PID[4]>PID[6]):
        PID[4]=PID[6]
    PID[5]=e
    if output>PID[7]:
        output=PID[7]
    elif output<-PID[7]:
        output=-PID[7]
    #print e
    PID[8]=output
    return output

def robot_control(lin, ang):
    velo=Twist()
    pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
    #speed=Twist(Vector3(vel, 0, 0), Vector3(0, 0, ang))
    velo.linear.x = -lin
    velo.angular.z= ang
    #print(-lin,ang)
    pub.publish(velo)

def colorFiltering(img): #yellow filter
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #hsv.shape[1]
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    filtered = cv2.bitwise_and(img, img, mask = mask) #return filtered yellow picture
    (thres, bw) = cv2.threshold(cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY)
    return bw
def circle_detect(img): #return image, x, y, r
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,2,50,param1=50,param2=30,minRadius=0,maxRadius=100)
    if circles is None:
        #print("No circle is detected")
        return cimg, -1,-1,-1
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
        #print("center:(",circles[0][0][0],",",circles[0][0][1],")")
        #print("radius:",circles[0][0][2])
        return cimg, circles[0][0][0], circles[0][0][1], circles[0][0][2]

def image_callback(msg):
    global val_sim, cv2_img
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2_img = cv2.blur(cv2_img, (10, 10))
        cv2_img,x,y,r = circle_detect(colorFiltering(cv2_img))
        #print(x, y, r)
        if(x!=-1):
            robot_control(pid(lin_pid, float(r)), pid(ang_pid, float(x)))
            #robot_control(1*(float(r)-200)/256,0.5*(float(x)-256.0))
        else:
            robot_control(0,0)
        cv2.imshow('filtered',cv2_img)
        cv2.waitKey(1)
    except CvBridgeError, e:
        print(e)

def main():
    print "ball tracker start"
    rospy.init_node('ball_track')
    image_topic = "/vrep/image"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
