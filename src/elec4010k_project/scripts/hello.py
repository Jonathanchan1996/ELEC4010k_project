#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
#from geometry_msgs.msg import *
def hello():
    pub = rospy.Publisher('tester', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
	#speed=Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
        hello_str="%s" % rospy.get_rostime()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        hello()
    except rospy.ROSInterruptException:
        pass
