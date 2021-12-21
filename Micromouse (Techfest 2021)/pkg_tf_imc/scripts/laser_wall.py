#! /usr/bin/env python

import re
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray

global reg

reg = Int16MultiArray()

def clbk_laser(msg):
    # 720 / 5 = 144
    regions = [
        0 if min(min(msg.ranges[0:71]), 0.16) >= 0.13 else 1,
        0 if min(min(msg.ranges[144:214]), 0.18) >= 0.18 else 1,
        0 if min(min(msg.ranges[288:359]), 0.16)>= 0.13 else 1,
    ]
    #rospy.loginfo(regions)
    reg.data = regions
    pub.publish(reg)


def main():
    global pub
    rospy.init_node('reading_laser')
    
    pub = rospy.Publisher('/laserWall', Int16MultiArray, queue_size=10)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    
    rospy.spin()


if __name__ == '__main__':
    main()
