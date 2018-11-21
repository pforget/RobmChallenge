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
    # Recupération de l'image depuis ROS et conversion au format BGR 8 bit par canal
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Taille de l'image : lignes, colonnes et nombre de canaux de couleurs
    (rows,cols,channels) = cv_image.shape

    # On dessine un cercle
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)


    # Accès aux pixels d'une colonne
    x = 40
    # for, accès lent
    for y in range(rows):
      cv_image[y, x, 0] = 255 # canal bleu a 255
    # slice, accès rapide
    cv_image[:, x, 1] = 0 # canal vert à 0

    # Access aux pixels d'une ligne
    y = 60
    # for, accès lent
    for x in range(cols): 
      cv_image[y, x, 1] = 255 # canal vert à 255
    # slice, accès rapide
    cv_image[y, :, 2] = 0 # canal rouge à 0

    # Affichage de la valeur d'un pixel
    rospy.loginfo("Le pixel (70,70) vaut %s", cv_image[70, 70])

    # Affichage simplifié dans une fenetre OpenCV
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # Conversion en image ROS et publication
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('example_image_rgb')
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
