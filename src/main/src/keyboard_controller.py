#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import *

import tty
import sys
import time
import termios
import select
import numpy as np
lin = 0.5
ang = 0.5

msg = """








ELEC4010k Project controller
============================
As video game
arrow:      Increase velo:
q w e             i
a s d           j k l
  x
where x is stop :p
j is angular Increase
l is angular decrease
exit: f
============================
"""


settings = termios.tcgetattr(sys.stdin)
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
def sign(x):
  return 1-(x<=0)
x = 0
last_velo=Twist()
level = 0.1
spd=0
yaw=0
def key_crt(x):
    global lin, ang, spd, yaw
    velo=Twist()
    if x=='w':
        spd = lin
        yaw = 0
    elif x=='s':
        spd =-lin
        yaw = 0
    elif x=='a':
        spd = 0
        yaw = ang
    elif x=='d':
        spd = 0
        yaw =-ang
    elif x=='e':
        spd = lin
        yaw =-ang
    elif x=='q':
        spd = lin
        yaw = ang
    elif x=='x':
        spd = 0
        yaw = 0
    else:
        if x=='i':
            lin=lin+level
            spd=sign(spd)*lin
        elif x=='k':
            lin=lin-level
            spd=sign(spd)*lin
        elif x=='j':
            ang=ang+0.1
            yaw=sign(yaw)*ang
        elif x=='l':
            ang=ang-0.1
            yaw=sign(yaw)*ang
        if lin<0:
            lin=0
            spd=0
        if ang<0:
            ang=0
            yaw=0
        lin = round(lin, 2)
        ang = round(ang, 2)
        spd = round(spd, 2)
        yaw = round(yaw, 2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    velo.linear.x=spd
    velo.angular.z=yaw
    return velo

def main():
    pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True) #my name
    rate = rospy.Rate(10) # 10hz
    print(msg)
    while(1):
        time.sleep(0.05)
        key = getKey()
        if(key=='f' or key=='c'):
            break
        else:
            print(msg)
            print (lin, ang)
            pub.publish(key_crt(key))
        #rospy.spin()

if __name__ == '__main__':
    try:
        main()
    #except rospy.ROSInterruptException:
    #    pass
    finally:
        print "End control"
