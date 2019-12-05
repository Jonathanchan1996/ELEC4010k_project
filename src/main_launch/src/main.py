#!/usr/bin/env python
import roslaunch
import os

os.system("cd ~/catkin_ws;roslaunch ecejonathan.launch")
print "ball tracking mode"
os.system("rosrun ball_track track.py")
print "main ended"
'''
package = 'image_detect'
executable = 'image_cv.py'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print process.is_alive()
process.stop()
'''
