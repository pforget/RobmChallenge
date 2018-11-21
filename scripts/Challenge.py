#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy


from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
from numpy import sin, cos, pi
from skimage.draw import line

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
import math

class Challenge:

    def __init__(self):
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber("image_topic",Image,self.image_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def odom_callback(self, odom):
		"""Callback du mise à jour de la vitesse désirée"""
		self.odom = odom

		# Publish speed command (in not zero for too long)
		vel_msg = Twist()
		vel_msg.linear.x = 0.5
		vel_msg.angular.z = 0
		self.cmd_pub.publish(vel_msg)


    def scan_callback(self,scan):
        """Todo callback du laser"""
        self.scan = scan
        
    def image_callback(self, data):
        self.image = data         #TODO

def challenge_node():
    rospy.init_node('challenge')
    challenge = Challenge()
    rospy.spin()

if __name__ == '__main__':
    try:
        challenge_node()
    except rospy.ROSInterruptException:
        pass

