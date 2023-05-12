#!/usr/bin/env python

from typing import List
import rospy
from rospy.topics import Publisher
import tf2_msgs.msg
import tf
import tf2_ros
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, PoseArray, Pose, Point ,Quaternion, Vector3 ,TransformStamped
from sensor_msgs.msg import JointState, LaserScan
from math import cos, sin, cosh, sinh, sqrt, tan , atan2
from std_msgs.msg import Duration, Time
import tf_conversions


class Smooth:
    
    def __init__(self,current_time):

        rospy.Subscriber("/tricycle_controller/cmd_vel", Twist , self.smoother_cb)
        self.pub = rospy.Publisher("/tricycle_controller/cmd_vel", Twist,self, queue_size = 10)

        self.prev_time = current_time
        self.cmd_vel = Twist().angular.z
        self.rate = rospy.Rate(50)


    def smoother_cb(self,msg):

        current_time_ = rospy.Time.now()
        dt = -1*(self.prev_time - current_time_).to_sec()
        # print("dt: ",dt)
        self.prev_time = current_time_

        while True:

            self.cmd_vel = msg.angular.z
            # List = []
            # List.append(self.cmd_vel)
            
            if (self.cmd_vel < 0.06):
                self.pub.publish(self.cmd_vel)
                

       
            self.rate.sleep()
        
        rospy.spin()





if __name__ == '__main__':
    rospy.init_node("Smooth",anonymous=True)
    currentTime = rospy.Time.now()
    try:
        x = Smooth(currentTime)
        # x.smoother_cb()
    except rospy.ROSInterruptException:
        pass