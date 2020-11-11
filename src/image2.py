#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)

    ## initialize a publisher to publish x-coordinates for blob centres
    self.x_centres_pub = rospy.Publisher("x_centres_topic", Float64MultiArray, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(3)
    self.joints = Float64MultiArray()
    self.joints.data = self.detect_blob_centre_xs(self.cv_image2)
    

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.x_centres_pub.publish(self.joints)
    except CvBridgeError as e:
      print(e)
  
  def detect_blob_centre_xs(self, image):

    a = self.pixel2meter(image)
    yX = a*self.detect_yellow_x(image)
    bX   = a*self.detect_blue_x(image)
    gX  = a*self.detect_green_x(image)
    rX   = a*self.detect_red_x(image)
    tX = a*self.detect_target_x(image)

    return np.array([bX-yX, gX-yX, rX-yX,tX-yX])
  
  def detect_red_x(self, image):
    red_blob = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    kernel = np.ones((5, 5), np.uint8)
    red_blob = cv2.dilate(red_blob, kernel, iterations=8)
    M = cv2.moments(red_blob)
    if(M['m00'] == 0):
      cX = self.detect_target_x(image)
    cX = int(M['m10'] / M['m00'])
    return cX

  def detect_yellow_x(self, image):
    yellow_blob = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    yellow_blob = cv2.dilate(yellow_blob, kernel, iterations=8)
    M = cv2.moments(yellow_blob)
    cX = int(M['m10'] / M['m00'])
    return cX

  def detect_blue_x(self, image):
    blue_blob = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    blue_blob = cv2.dilate(blue_blob, kernel, iterations=8)
    M = cv2.moments(blue_blob)
    cX = int(M['m10'] / M['m00'])
    return cX

  def detect_green_x(self, image):
    green_blob = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    green_blob = cv2.dilate(green_blob, kernel, iterations=8)
    M = cv2.moments(green_blob)
    if(M['m00'] == 0):
      cX = self.detect_target_x(image)
      return cX
    cX = int(M['m10'] / M['m00'])
    return cX

    # Detecting the centre of the blue circle
  def detect_blue(self,Image):
    mask = cv2.inRange(Image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if(M['m00'] == 0):
        cx = 0
        cy = 0
        return np.array([cx,cy])
        #in case it overlap with another image, return 0,0
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_target_x(self,image):
    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (11, 43, 46), (25, 255, 255))
    kernel = np.ones((8, 8), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if np.where(mask != 0)[0].shape[0] == 0:
      return self.detect_red(image)
    M = cv2.moments(mask)
    cX = int(M['m10'] / M['m00'])
    return cX

  # Detecting the centre of the yellow circle
  def detect_yellow(self,Image):
    mask = cv2.inRange(Image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if(M['m00'] == 0):
        cx = 0
        cy = 0
        return np.array([cx,cy])
        #in case it overlap with another image, return 0,0
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
    circle1Pos = self.detect_yellow(image)
    circle2Pos = self.detect_blue(image)
      # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos)**2)
    return 2.5 / np.sqrt(dist)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


