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
    self.joint_pos2 = rospy.Publisher("joint_pos2", Float64MultiArray, queue_size = 1)

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
    #im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(3)
    self.joints = Float64MultiArray()
    self.joints.data = self.detect_joint_pos2(self.cv_image2)
   

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.joint_pos2.publish(self.joints)
     
    except CvBridgeError as e:
      print(e)
  
 
  
  def detect_target(self, image):
      h, w, ch = image.shape
      result = np.zeros((h, w, ch), dtype=np.uint8)
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      target_lower = np.array([11, 43, 46])
      target_upper = np.array([25, 255, 255])
      #orange detection
      target_mask = cv2.inRange(gray, target_lower, target_upper)

      #cv2.imshow("target_mask1", target_mask)
      contours, hierarchy = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      for cnt in range(len(contours)):
          cv2.drawContours(result, contours, cnt, (0, 255, 0), 2)
          # approximation of contours
          epsilon = 0.01 * cv2.arcLength(contours[cnt], True)
          approx = cv2.approxPolyDP(contours[cnt], epsilon, True)
          corners = len(approx)
          #if corner > 10 it can only be sphere
          if corners >= 10:             
              mm = cv2.moments(contours[cnt])
              if(mm['m00']==0):
                
                return np.array([0,0])
              cx = int(mm['m10'] / mm['m00'])
              cy = int(mm['m01'] / mm['m00'])
              
              #print(cx,cy,'sphere')
              return np.array([cx,cy])
      
      return np.array([0,0])

  
  
  def detect_red(self,Image):
      # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(Image, (0, 0, 100), (0, 0, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
    M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
    if(M['m00'] == 0):
        
        
        return np.array([0,0])
        
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    
    
    
    return np.array([cx, cy])
  
    
            
  
  # Detecting the centre of the green circle
  def detect_green(self,Image):
    mask = cv2.inRange(Image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if(M['m00'] == 0):
        
        return np.array([0,0])
        
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    
   
    return np.array([cx, cy])

  # Detecting the centre of the blue circle
  def detect_blue(self,Image):
    mask = cv2.inRange(Image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if(M['m00'] == 0):
        
        return np.array([0,0])
        
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    
    
    return np.array([cx, cy])

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

    
   # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
    circle1Pos = self.detect_yellow(image)
    circle2Pos = self.detect_blue(image)
      # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos)**2)
    return 2.5 / np.sqrt(dist)

  # Calculate the relevant joint angles from the image

  def detect_joint_pos2(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob
    center = a * self.detect_yellow(image)
    circle1Pos1 = a * self.detect_blue(image) 
    circle2Pos1 = a * self.detect_green(image) 
    circle3Pos1 = a * self.detect_red(image)
    targetpos = a * self.detect_target(image)
    # transfer the x,y cordinate respect to center[0,0] yellow
    circle1Pos = [circle1Pos1[0] - center[0], center[1] - circle1Pos1[1]]
    circle2Pos = [circle2Pos1[0] - center[0], center[1] - circle2Pos1[1]]
    circle3Pos = [circle3Pos1[0] - center[0], center[1] - circle3Pos1[1]]
    target = [targetpos[0] - center[0], center[1] - targetpos[1]]

    #print(circle1Pos,circle2Pos,circle3Pos,target)
    return np.array(circle1Pos + circle2Pos + circle3Pos + target)
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
