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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a publisher to send joint position detected by camera 1
    self.joints_pub1 = rospy.Publisher('joint_angle1', Float64MultiArray, queue_size=10)
    
    ## initialize a publisher to send joint length detected by camera 1
    self.length_pub1 = rospy.Publisher('joint_length1',Float64MultiArray, queue_size=10)
    
    ## initialize a publisher to send joint position detected by camera 1
    self.pos_pub1 = rospy.Publisher('joint_pos1',Float64MultiArray,queue_size=10)

    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
  
    # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    a = self.detect_joint_angles(self.cv_image1)
    l = self.detect_joint_lengths(self.cv_image1)
    joints_pos_data = self.detect_joint_pos(self.cv_image1)
    cv2.imshow('window', self.cv_image1)
    cv2.waitKey(1)
    if(any([np.absolute(x) > 12 for x in joints_pos_data])):
      return

    self.joints = Float64MultiArray()
    self.joints.data = a
    self.length = Float64MultiArray()
    self.length.data = l
    self.joints_pos = Float64MultiArray()
    self.joints_pos.data = joints_pos_data

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joints_pub1.publish(self.joints)
      self.length_pub1.publish(self.length)
      self.pos_pub1.publish(self.joints_pos)
    except CvBridgeError as e:
      print(e)

  
  
  
  def detect_target(self,image):
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (11, 43, 46), (25, 255, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((8, 8), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # if the target is hidden, then make it to the centre of red ball
        if np.where(mask != 0)[0].shape[0] == 0:
            return self.detect_red(image)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return [cx, cy]
  
  
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
        cx = 0
        cy = 0
        return np.array([cx,cy])
        #in case it overlap with another image, return 0,0
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
        cx = 0
        cy = 0
        return np.array([cx,cy])
        #in case it overlap with another image, return 0,0
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
        cx = 0
        cy = 0
        return np.array([cx,cy])
        #in case it overlap with another image, return 0,0
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
  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1
    ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja1, ja2, ja3])
    
  # Calculate the relevant joint angles from the image
  def detect_joint_lengths(self,image):
    l = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    center = l * self.detect_yellow(image)
    circle1Pos = l * self.detect_blue(image) 
    circle2Pos = l * self.detect_green(image) 
    circle3Pos = l * self.detect_red(image)
    # Solve using trigonometry
    jl1 = np.sqrt((center[0]- circle1Pos[0])**2+(center[1] - circle1Pos[1])**2)
    jl2 = np.sqrt((circle1Pos[0]-circle2Pos[0])**2+(circle1Pos[1]-circle2Pos[1])**2)
    jl3 = np.sqrt((circle2Pos[0]-circle3Pos[0])**2+(circle2Pos[1]-circle3Pos[1])**2)
    return np.array([jl1, jl2, jl3])
  
  def detect_joint_pos(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob
    center = self.detect_yellow(image)
    # Calculate the relative position of each joint with respect to the yellow joint
    circle1Pos = [j-i for i, j in zip(self.detect_blue(image), center)]
    circle2Pos = [j-i for i, j in zip(self.detect_green(image), center)]
    circle3Pos = [j-i for i, j in zip(self.detect_red(image), center)]
    target = [j - i for i, j in zip(self.detect_target(image), center)]
    return a * np.array(circle1Pos + circle2Pos + circle3Pos + target)

    
    


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


