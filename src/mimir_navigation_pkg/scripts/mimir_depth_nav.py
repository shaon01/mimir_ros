#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

def depthDataMonitorCB(data):
    distData = data.data
    max_dist = max(distData)
    #maxIndexList = [index for index,value in enumerate(distData) if value==max(distData)]
    for idx, val in enumerate(distData):
        rospy.loginfo(rospy.get_caller_id() + "I got distance: %d, at index: %d", val,idx)
        #rospy.loginfo('index of value %d',)

def listener():
    rospy.init_node('loki_nav',anonymous=False)
    rospy.Subscriber("/camera/depth/image_rect_raw/compressedDepth", CompressedImage, depthDataMonitorCB)
    rospy.spin()

if __name__ == '__main__':
    listener()
