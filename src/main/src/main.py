#!/usr/bin/env python
import os
import time
print "ELEC4010k Project Start"
os.system("roslaunch main step1.launch")
time.sleep(1)
print "Yellow Ball Tracking Mode"
#os.system("cd ~/catkin_ws;roslaunch main step2.launch")
os.system("rosrun ball_track track.py")
print "main ended"
