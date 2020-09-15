#!/usr/bin/env python
import rospy 
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist

pub = rospy.Publisher('Loki_drive_direction', String, queue_size=10)

def nevigationCbk(direction):
    rospy.loginfo('got messes linearX : %f, angularZ: %f',direction.linear.x, direction.angular.z)
    if direction.linear.x > 0:
        pub.publish("Forward")
    elif direction.linear.x < 0:
        pub.publish("Backward")
    else:
        pub.publish("Stop")
def listener():
    rospy.init_node("loki_drive",anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, nevigationCbk)
    rate = rospy.Rate(10)
    rospy.spin()
    rospy.loginfo('exiting node')

listener()
