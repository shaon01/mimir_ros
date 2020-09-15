#!/usr/bin/env python
import rospy 
from std_msgs.msg import String
import time

directionList = ["Forward","L_shift","R_shift","Backward","Stop"]

def lokiDemo():
    pub = rospy.Publisher('Loki_drive_direction', String, queue_size=10)
    rospy.init_node("demoDriver",anonymous=True)
    rate = rospy.Rate(10)

    for idx, val in enumerate(directionList):
        pub.publish(val)
        time.sleep(4)
        rate.sleep()


if __name__ == '__main__':
    lokiDemo()
