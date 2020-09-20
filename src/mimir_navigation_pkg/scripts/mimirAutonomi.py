#!/usr/bin/env python
import rospy 
from std_msgs.msg import String,Int8
import time

curDirection = ''
oldDirection = ''

pub = rospy.Publisher('Loki_drive_direction', String, queue_size=10)

def autoNaviCBK(distance):
    global oldDirection
    if distance.data > 0:
        disToObj = distance.data
        rospy.loginfo('current distance : %d',disToObj)
        if disToObj > 30:
            curDirection = 'Forward'
        else:
            curDirection = 'Stop'
        if curDirection != oldDirection:
            pub.publish(curDirection)
            oldDirection = curDirection
    

    
def rosInit():
    rospy.init_node('mimir_auto_nav',anonymous=True)
    rospy.Subscriber('/Distance_sensor', Int8, autoNaviCBK)
    rate = rospy.Rate(10)
    rospy.spin()

rosInit()