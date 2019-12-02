#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import *

import tty
import sys
import termios

import numpy as np


def TesterCallback(data):
    rospy.loginfo(rospy.get_caller_id()+" : %s", data.data)

def getTester():
    rospy.Subscriber("tester", String, TesterCallback)

def lidarCallback(msg):
    #print "Lidar IT"
    T=1;
    #rospy.loginfo("Lidar : %f", msg.ranges[0])

def getLidar():
    rospy.Subscriber("/vrep/scan", LaserScan, lidarCallback)
def cameraCallback(msg):
    T=1;
    #print "Camera IT"

def getCamera():
    rospy.Subscriber("/vrep/image", Image, cameraCallback)
x = 0
def key_crt():
    orig_settings=termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    x=sys.stdin.read(1)[0]
    if x=='w':
        print x
        velo=Twist(Vector3(5, 0, 0), Vector3(0, 0, 0))
    elif x=='s':
        print x
        velo=Twist(Vector3(-5, 0, 0), Vector3(0, 0, 0))
    elif x=='a':
        print x
        velo=Twist(Vector3(0, 0, 0), Vector3(0, 0, -5))
    elif x=='d':
        print x
        velo=Twist(Vector3(0, 0, 0), Vector3(0, 0, 5))
    elif x=='x':
        print x
        velo=Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    return velo

def main():
    pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
    rospy.init_node('main', anonymous=True) #my name
    #getTester()
    getLidar()
    getCamera()
    rate = rospy.Rate(10) # 10hz
    print "init ok\n"
    while not rospy.is_shutdown():

        #velo=Twist(Vector3(0, 0, 0), Vector3(0, 0, 5)) #linear, angular
        pub.publish(key_crt())
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
