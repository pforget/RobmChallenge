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
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
import math

def getI2P():
    R = np.eye(3)
    T = np.array([0,0,0])
    rTw = np.vstack((np.hstack((R, T[:,np.newaxis])),
                 np.array([0, 0, 0, 1])))
    # Transformation repère robot -> repère caméra : Zc=Xr, Xc=-Yr, Yc=-Zr
    cTr = np.array([[0, -1, 0, 0],
                [0, 0, -1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1]])

    theta = 10.0 * pi / 180.0
    Rcam = np.array([[cos(theta),  0, sin(theta)],
                    [0,       1, 0     ],
                    [-sin(theta), 0, cos(theta)]])
    Tcam = np.array([0,0,0.15])
    

    camTr = np.vstack((np.hstack((Rcam.T, np.dot(-Rcam.T , Tcam[:,np.newaxis]))),
                 np.array([0, 0, 0, 1])))

    cTw = np.dot(cTr, np.dot(camTr, rTw))
    # Le plan considéré est z=0 dans le repère monde, la transformation est donc l'identité
    wTp = np.eye(4)
    # Matrice Pp
    Pp = np.eye(4)[:, [0,1,3]]

    K = np.array([[265.23, 0.0, 160.5],
              [0.0, 265.23, 120.5],
              [0.0, 0.0, 1.0]])

    Pi = np.eye(3,4)
    # Matrice de transformation image -> plan
    return np.dot(wTp, np.dot(Pp, np.linalg.inv(np.dot(K, np.dot(Pi, np.dot(cTw, np.dot(wTp, Pp)))))))

def getWorldPos(u,v):
    point = [u,v,1]    
    result = np.dot(I2P, point)
    result /= result[3]
    return result[0],result[1]

def world_to_grid_coord(x, y, map_info):
    res = map_info.resolution
    x0 = int((x - map_info.origin.position.x)/res)
    y0 = int((y - map_info.origin.position.y)/res)

    height = map_info.height
    width = map_info.width
    ifValue = x0  > width
    ifValue2 =  y0 > height
    if ifValue or ifValue2 or x0 < 0 or y0 < 0:
        return (None,None)
    else :
        return (x0, y0) 

def quaternion_msg_from_yaw(yaw):
	q = quaternion_from_euler(0.0, 0.0, yaw)
	return Quaternion(*q)

class Challenge:

    def __init__(self):
        global map_info
        global I2P
        
        self.firstCall = True
        I2P = getI2P()
        self.bridge = CvBridge()
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber("camera/image",Image,self.image_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_map = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        

        self.map = OccupancyGrid()
        self.map.header.frame_id = "odom"
        map_info = self.map.info
        map_info.resolution = 0.05
        map_info.width = 384
        map_info.height = 384
        map_info.origin.position.x = -0.5 * map_info.resolution * map_info.width
        map_info.origin.position.y = -0.5 * map_info.resolution * map_info.height
        map_info.origin.orientation = quaternion_msg_from_yaw(0.0)
		# - initialisation des données de la carte
        self.map.data = np.ones( map_info.height*map_info.width, dtype="int8" ) * 50
		

    def getTransitionGrid(self):

        # Taille de l'image : lignes et colonnes
        (rows,cols) = self.image.shape
        self.transition_grid = [None]*rows*cols
        for y in range(rows) :
            for x in range(cols) :
                self.transition_grid[x+ y*cols] = world_to_grid_coord( *getWorldPos(x,y), map_info=map_info )
        # self.pub_transgrid.publish(self.transition_grid)
        # rospy.loginfo(self.transition_grid[cols*200:cols*201])
        self.firstCall = False

    def updateAndPublishGrid(self):
        (rows,cols) = self.image.shape 
              
        for y in range(rows/2,rows) :
            for x in range(cols) :
                (u,v) = self.transition_grid[x + y*cols]
                if u is not None :
                    if self.image[y,x] > 200 :
                       self.map.data[u+v*map_info.width] = 100
                    # else:
                    #    self.map.data[u+v*map_info.width] = 0
        self.pub_map.publish(self.map)  


        

    def odom_callback(self, odom):
		"""Callback du mise à jour de la vitesse désirée"""
		self.odom = odom

		# Publish speed command (in not zero for too long)
		vel_msg = Twist()
		vel_msg.linear.x = 0.001
		vel_msg.angular.z = 0
		self.cmd_pub.publish(vel_msg)


    def scan_callback(self,scan):
        """Todo callback du laser"""
        self.scan = scan

        
    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        if self.firstCall :
            self.getTransitionGrid()
        else :
            #self.pub_map.publish(self.map)  
            self.updateAndPublishGrid()
          

          
def challenge_node():
    rospy.init_node('challenge')
    challenge = Challenge()
    rospy.spin()

if __name__ == '__main__':
    try:
        challenge_node()
    except rospy.ROSInterruptException:
        pass

