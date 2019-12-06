#!/usr/bin/env python
import os

print "ELEC4010k Project Start"
os.system("cd ~/catkin_ws;roslaunch main step1.launch")
print "Yellow Ball Tracking Mode"
os.system("cd ~/catkin_ws;roslaunch main step2.launch")
print "main ended"
