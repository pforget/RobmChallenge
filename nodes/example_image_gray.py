#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

  def callback(self,data):
    # Recupération de l'image depuis ROS et conversion en niveau de gris 8 bit
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    # Taille de l'image : lignes et colonnes
    (rows,cols) = cv_image.shape

    # On dessine un cercle (en blanc = 255)
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)


    # Accès aux pixels d'une colonne
    # for, accès lent à la colonne x=40
    for y in range(rows):
      cv_image[y, 40] = 255 # en blanc
    # slice, accès rapide à la colonne x=41
    cv_image[:, 41] = 192 # en gris clair

    # Access aux pixels d'une ligne
    # for, accès lent à la ligne y=60
    for x in range(cols): 
      cv_image[60, x] = 0 # en noir
    # slice, accès rapide à la ligne y=61
    cv_image[61, :] = 64 # en gris foncé

    # Affichage de la valeur d'un pixel
    rospy.loginfo("Le pixel (70,70) vaut %s", cv_image[70, 70])

    # Affichage simplifié dans une fenetre OpenCV
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # Conversion en image ROS et publication
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('example_image_gray')
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
