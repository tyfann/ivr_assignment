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
    self.joint_pos3 = rospy.Publisher("joint_pos3", Float64MultiArray, queue_size = 10)

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
    
    self.joints = Float64MultiArray()
    
    self.joints.data = self.circlep(self.findcircle(self.cv_image2))
    #print((self.findcircle(self.cv_image2)))

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.joint_pos3.publish(self.joints)
     
    except CvBridgeError as e:
      print(e)
  def findcircle(self,image):
   
   

     #for black
     mask = cv2.inRange(image, (0, 0, 0), (180, 255, 46))
     img = cv2.medianBlur(mask, 5)
     #cv2.imshow("detected circles", img)
     cimg = image.copy()

     circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 10, np.array([]), 100, 7, 6, 16)
     if circles is not None:
        a, b, c = circles.shape
        #print(circles)
        for i in range(b):
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3, cv2.LINE_AA)  # draw center of circle

        cv2.imshow("detected circles", cimg)
        cv2.waitKey(1)
     return circles
  def circlep(self,circle):
    a, b, d = circle.shape
    c = np.array([],dtype='float64')
    
    for i in range(b):
      c = np.append(c,circle[0,i])
        
    return c
 
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
