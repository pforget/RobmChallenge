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

# Extract yaw from a Quaternion message
def yaw_from_quaternion_msg( q ):
	quat_array = [q.x, q.y, q.z, q.w]
	yaw = euler_from_quaternion(quat_array)[2]
	return yaw

def quaternion_msg_from_yaw(yaw):
	q = quaternion_from_euler(0.0, 0.0, yaw)
	return Quaternion(*q)

class Challenge:

    def __init__(self):
        global map_info
        global I2P
        
        self.isReady = True
        self.firstCall = True
        self.avance = 11
        self.avanceAngle = 6
        I2P = getI2P()
        self.bridge = CvBridge()
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber("camera/image",Image,self.image_callback, queue_size=1,)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_map = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        

        self.map = OccupancyGrid()
        self.map.header.frame_id = "odom"
        map_info = self.map.info
        map_info.resolution = 0.025
        map_info.width = 800
        map_info.height = 800
        map_info.origin.position.x = -0.5 * map_info.resolution * map_info.width
        map_info.origin.position.y = -0.5 * map_info.resolution * map_info.height
        map_info.origin.orientation = quaternion_msg_from_yaw(0.0)
		# - initialisation des données de la carte
        self.map.data = np.ones( map_info.height*map_info.width, dtype="int8" ) * 50
        self.map.data[:] = 1
        
        
        

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
        imgXD, imgYD = self.transition_grid[0]
        imgXF, imgYF = self.transition_grid[310]        
        imgXC = int((imgXD + imgXF) /2)
        imgYC = int((imgYD + imgYF) /2)
        self.imgGC = imgYC + imgXC * map_info.width

        

    def updateAndPublishGrid(self): 
        
        # self.map.data[:] = 0
        x = self.odom.pose.pose.position.x
        y = -self.odom.pose.pose.position.y
        theta = yaw_from_quaternion_msg(self.odom.pose.pose.orientation)
        # rospy.loginfo(theta);
        cosT = math.cos(theta)
        sinT = math.sin(theta)
        xc, yc = world_to_grid_coord(0,0, map_info)
        centralPoint = yc + xc * map_info.width 

        x0,y0 = world_to_grid_coord(x,y, map_info)
        # rospy.loginfo("x: %s y: %s x0 %s y0 %s xc %s yc %s", x,y,x0, y0,xc,yc)
        posRob = y0 + x0 * map_info.width

        # rospy.loginfo("update %s", posRob)
        
        # self.map.data[centralPoint] = 600

        (rows,cols) = self.image.shape 
        
        difRobo = posRob - centralPoint
        decalage = -25 * map_info.width
        dx,dy = 0 ,0
        imgCor = self.imgGC - centralPoint - decalage
        # rospy.loginfo("robo %s pos1 %s difRobo %s final %s" , posRob, pos1, difRobo, pos1  + difRobo)
        # y = rows/2
        for y in range(rows/2, rows -1,10):
            for x in range(0,310,1):
                point = x + y * cols
                img= self.image[y,x]


                a,b = self.transition_grid[point]
                ox, oy = xc, yc
                px, py = a , b
                qx = int(ox + cosT * (px - ox) - sinT *(py - oy))
                qy = int(oy + sinT * (px - ox) + cosT *(py - oy))
                aF = qx
                bF = qy
                pos = (map_info.width -bF) + aF * map_info.width
                posFinal = pos + difRobo - imgCor
                # rospy.loginfo("x: %s y: %s pos: %s image %s" , x,y , pos, img)
                if img > 200:
                    self.map.data[posFinal] = 50
                else :
                    if self.map.data[posFinal] == 1 :
                        self.map.data[posFinal] = 100


        # self.map.data[posRob] = 400
        self.pub_map.publish(self.map)
        


    def odom_callback(self, odom):
        """Callback du mise à jour de la vitesse désirée"""
        self.odom = odom
        if self.firstCall:
            return
        l, a = self.path()
        # Publish speed command (in not zero for too long)
        vel_msg = Twist()
        vel_msg.linear.x = l
        vel_msg.angular.z = a
        self.cmd_pub.publish(vel_msg)

    def path(self):
        x = self.odom.pose.pose.position.x
        y = -self.odom.pose.pose.position.y
        theta = yaw_from_quaternion_msg(self.odom.pose.pose.orientation)
        x0,y0 = world_to_grid_coord(x,y, map_info)
        posRob = int(y0 + x0 * map_info.width)
        cosT = math.cos(-theta)
        sinT = math.sin(-theta)


        ox, oy = x0, y0
        px, py = x0 + self.avance, y0
        ax, ay = x0 + self.avanceAngle, y0

        qx = int(ox + cosT * (px - ox) - sinT * (py - oy))
        qy = int(oy + sinT * (px - ox) + cosT * (py - oy))

        posf = qy + qx * map_info.width
        # self.map.data[posf] = 600

        xx,yy = line(x0 , y0, qx, qy)
        posTab = yy + xx * map_info.width

        thetaL = yaw_from_quaternion_msg(self.odom.pose.pose.orientation) +0.8
        cosTL = math.cos(-thetaL)
        sinTL = math.sin(-thetaL)
        qxL = int(ox + cosTL * (ax - ox) - sinTL * (ay - oy))
        qyL = int(oy + sinTL * (ax - ox) + cosTL * (ay - oy))
        posfL = qyL + qxL * map_info.width
        xxL,yyL = line(x0 , y0, qxL, qyL)
        posTabL = yyL + xxL * map_info.width

        thetaR = yaw_from_quaternion_msg(self.odom.pose.pose.orientation) -0.8
        cosTR = math.cos(-thetaR)
        sinTR = math.sin(-thetaR)
        qxR = int(ox + cosTR * (ax - ox) - sinTR * (ay - oy))
        qyR = int(oy + sinTR * (ax - ox) + cosTR * (ay - oy))
        posfR = qyR + qxR * map_info.width
        xxR,yyR = line(x0 , y0, qxR, qyR)
        posTabR = yyR + xxR * map_info.width

       # rospy.loginfo("left %s right %s center %s ",self.map.data[posTabL] == 50,self.map.data[posTabR]== 50, self.map.data[posTab]== 50)

        vit = 0
        ang = 0
        
        listDroit = self.map.data[posTab]
        listL = self.map.data[posTabL]
        listR = self.map.data[posTabR]



        if not any(self.map.data[posTab] == 50):
            vit = 0.25
            ang = 0
            rospy.loginfo("center1 %s %s", vit, ang)
        else :
            if not any(listL == 50):
                vit = 0.0
                ang = 0.2
                rospy.loginfo("left1 %s %s", vit, ang)
            else :
                if not any(listR == 50):
                    vit = 0.0
                    ang = -0.2
                    rospy.loginfo("right1 %s %s", vit, ang)
                else :
                    indexs = np.where(listDroit == 50)
                    indexR = np.where(listR == 50)
                    indexL = np.where(listL == 50)

                  
                    a = indexs[0]
                    ir = indexR[0]
                    il = indexL[0]

                    if a[0] > ir [0] :
                        if a[0] > il[0] :
                            vit = 0.25
                            ang = 0
                            rospy.loginfo("center2 %s %s", vit, ang)
                        else :
                            vit = 0.0
                            ang = 0.25
                            rospy.loginfo("left2 %s %s", vit, ang)
                    else :
                        if ir[0] > il[0] :
                            vit = 0.0
                            ang = -0.25
                            rospy.loginfo("right3 %s %s", vit, ang)
                        else :
                            rospy.loginfo("gauche %s, droite %s",il[0], ir[0])
                            vit = 0.0
                            ang = 0.25
                            rospy.loginfo("left3 %s %s", vit, ang)
                           

        

        # self.map.data[posTab] = 300
        # self.map.data[posTabL] = 100
        # self.map.data[posTabR] = 200
        # rospy.loginfo(self.map.data[posTab])
        # if any(self.map.data[posTab] == 50) :
            
        #     if any(self.map.data[posTabL] == 50):
        #         vit = 0.20
        #         ang = 1
        #         rospy.loginfo("left %s %s", vit, ang)
        #     else:
                
        #         if any(self.map.data[posTabR] == 50):
        #             vit = 0.20
        #             ang = -1
        #             rospy.loginfo("right %s %s", vit, ang)
        #         else:
        #             rospy.loginfo("fuck")
            # if self.tryleft:
            # else: 
            #     if self.tryRight:
            #         vit = 0.20
            #         ang = -1
            #         rospy.loginfo("right %s %s", vit, ang)
            #     else:
            #         vit = 0
            #         ang = 0
        # else:
        #     vit = 0.25
        #     ang = 0.0
        #     rospy.loginfo("none %s %s", vit, ang)


        rospy.loginfo("publish")
        self.pub_map.publish(self.map)

        return vit, ang
    
    def scan_callback(self,scan):
        """Todo callback du laser"""
        self.scan = scan

        
    def image_callback(self, data):
        ti = data.header.stamp
        to = self.odom.header.stamp
        # rospy.loginfo("image: %s odom: %s %s" , ti, to, ti > to)
        if(ti < to):
            return
        
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
        if self.firstCall :
            self.getTransitionGrid()
        else :
            # self.pub_map.publish(self.map)  
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
