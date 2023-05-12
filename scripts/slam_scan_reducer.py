#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_cb(data):
    data.ranges = data.ranges[1:]
    pub_scan.publish(data)

rospy.init_node('slam_scan_pub')

pub_scan = rospy.Publisher('/scan_unified_fixed', LaserScan, queue_size=150)
rospy.Subscriber("/scan_unified", LaserScan, scan_cb)

rospy.spin()