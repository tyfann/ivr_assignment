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
    self.joint_sub2 = rospy.Subscriber('joint_pos3', Float64MultiArray, self.callback2)
    ## initialize a publisher to send joint position detected by camera 1
    self.pos_pub1 = rospy.Publisher('joint_pos1',Float64MultiArray,queue_size=10)
   
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.red = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    self.blue = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    self.yellow = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    self.green = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    
    self.red2 = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    self.blue2 = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    self.yellow2 = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    self.green2 = np.array([0.0,0.0,0.0,0.0,0.0],dtype='float64')
    
    self.circles2 = None
  def callback2(self,pos):
    self.circles2 = pos.data
    
    #print(self.circles2)
    # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

   
    #cv2.imshow('window', self.cv_image1)
    #cv2.waitKey(1)

    #joints_pos_data = self.detect_joint_pos(self.cv_image1)
    #if(any([np.absolute(x) > 12 for x in joints_pos_data])):
    #  return
    circles = self.findcircle(self.cv_image1)
    self.store(circles)
    self.verify(circles)
   
    if self.circles2 is not None and circles is not None:
      circles = self.findcircle(self.cv_image1)
      self.store(circles)
      self.verify(circles)
      
      self.store2(self.circles2)
      self.verify2(self.circles2)
      
      self.detect_joint_angles(self.detect_joint_pos())
   


    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))

      #self.pos_pub1.publish(self.joints_pos)
      
    except CvBridgeError as e:
      print(e)
  def detect_joint_angles(self,pos):
    blue = np.array([pos[0],pos[1],pos[2]])
    green = np.array([pos[3],pos[4],pos[5]])
    red = np.array([pos[6],pos[7],pos[8]])
    
    theta2 = np.arctan(-green[1]/(green[2]-2.5))
    theta3 = np.arctan((-np.sin(theta2)*green[0]/green[1]))
      
      
    tocenterG = np.sqrt((green[0] - blue[0])**2 + (green[1] - blue[1])**2)
    tocenterR = np.sqrt((red[0] - blue[0])**2 + (red[1] - blue[1])**2)
    distrb = red-blue
    distgb = green - blue
    distrg = red - green
    
    theta4 = np.arccos((distgb).dot(distrg)/(np.linalg.norm(distgb) * np.linalg.norm(distrg)))
    
    if(theta2>0 and tocenterG*distrb[2] > tocenterR*distgb[2]):
      theta4 = -theta4
    else:
      if(theta2<0 and tocenterG*distrb[2] < tocenterR*distgb[2]):
        theta4 = -theta4
    
    print(theta2,theta3,theta4)
    return np.array([theta2,theta3,theta4])    
  
  def findcircle(self,image):
   
   

     #for black
     mask = cv2.inRange(image, (0, 0, 0), (180, 255, 46))
     img = cv2.medianBlur(mask, 5)
     #cv2.imshow("detected circles", img)
     cimg = image.copy()
     circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 10, np.array([]), 100, 6, 6, 16)
     if circles is not None:
        a, b, c = circles.shape
        #print(circles)
        for i in range(b):
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(cimg, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3, cv2.LINE_AA)  # draw center of circle

        cv2.imshow("detected circles", cimg)
        cv2.waitKey(1)
     #print(circles)
     return circles
  
  def store(self,circles):

    if circles is not None and self.yellow[0] == 0.0:
        a, b, c = circles.shape
       
        z = np.array([0.0,0.0,0.0,0.0],dtype='float64')
        z = [circles[0,0,1],circles[0,1,1],circles[0,2,1],circles[0,3,1]]    
        
        for i in range(b):
          if(circles[0,i,1] == np.max(z)):
            
            self.yellow = [circles[0,i,0],circles[0,i,1],circles[0,i,2],circles[0,i,0],circles[0,i,1]]
            z[i] = 0.0
            break
        for i in range(b):
          if(circles[0,i,1] == np.max(z)):
            self.blue = [circles[0,i,0],circles[0,i,1],circles[0,i,2],circles[0,i,0],circles[0,i,1]]
            
            z[i] = 0.0
            break
        for i in range(b):    
          if(circles[0,i,1] == np.max(z)):
            self.green = [circles[0,i,0],circles[0,i,1],circles[0,i,2],circles[0,i,0],circles[0,i,1]]
            
            z[i] = 0.0
            break
        for i in range(b):
          if(circles[0,i,1] == np.max(z)):
            self.red = [circles[0,i,0],circles[0,i,1],circles[0,i,2],circles[0,i,0],circles[0,i,1]]
            break
        #print(self.yellow,self.blue,self.green,self.red) 
  def store2(self,circles):
        
        
        
    if circles is not None and self.yellow2[0] == 0.0:
        
       
        z = np.array([0.0,0.0,0.0,0.0],dtype='float64')
        z = [circles[1],circles[4],circles[7],circles[10]]    
        
        for i in range(12):
          if(circles[i] == np.max(z)):
            
            self.yellow2 = [circles[i-1],circles[i],circles[i+1],circles[i-1],circles[i]]
            z[round((i-1)/3)] = 0.0
            break
        for i in range(12):
          if(circles[i] == np.max(z)):
            self.blue2 = [circles[i-1],circles[i],circles[i+1],circles[i-1],circles[i]]
            
            z[round((i-1)/3)] = 0.0
            break
        for i in range(12):    
          if(circles[i] == np.max(z)):
            self.green2 = [circles[i-1],circles[i],circles[i+1],circles[i-1],circles[i]]
            
            z[round((i-1)/3)] = 0.0
            break
        for i in range(12):
          if(circles[i] == np.max(z)):
            self.red2 = [circles[i-1],circles[i],circles[i+1],circles[i-1],circles[i]]
            break
        #print(self.yellow2,self.blue2,self.green2,self.red2) 
  def verify2(self,circles):
    
    x = []
    z = []
    r = []
    mr = []
    mr2 = []
    mg = []
    mg2 = []
    countr = 0
    
    countg = 0
    
    b = round(len(circles)/3)
    for i in range(b):
      
      x.append(circles[i*3])
      z.append(circles[i*3+1])
      r.append(circles[i*3+2])
    for i in range(b):
      mr.append((x[i] - self.red2[3])**2 + (z[i] - self.red2[4])**2 + (r[i] - self.red2[2])**2)
    for i in range(b):
      mg.append((x[i] - self.green2[3])**2 + (z[i] - self.green2[4])**2 + (r[i] - self.green2[2])**2)  
    for i in range(b):
      if(r[i]*0.7 < self.red2[2] and self.red2[2] < r[i] *1.3 and mr[i] < 100 and mr[i] == np.min(mr) and countr == 0 or (r[i]*0.9 < self.red2[2] and self.red2[2] < r[i] * 1.1 and countr == 0 and mr[i] == np.min(mr))):
        self.red2[0] = self.red2[3]
        self.red2[1] = self.red2[4]
        self.red2[3] = x[i]
        self.red2[4] = z[i]
        countr = countr +1
    if(countr == 0):
      rx = self.red2[3]*2 - self.red2[0]
      rz = (self.red2[4]*2 - self.red2[1] + self.red2[4])/2
      self.red2[0] = self.red2[3]
      self.red2[1] = self.red2[4]
      self.red2[3] = rx
      self.red2[4] = rz
    #print(self.red)  
    print(countr)    
    
    for i in range(b):
      if(r[i]*0.7 < self.green2[2] and self.green2[2] < r[i] *1.3 and mg[i] < 100 and mg[i] == np.min(mg) and countg == 0 or (r[i]*0.9 < self.green2[2] and self.green2[2] < r[i] * 1.1 and countg == 0 and mg[i] == np.min(mg))):
        self.green2[0] = self.green2[3]
        self.green2[1] = self.green2[4]
        self.green2[3] = x[i]
        self.green2[4] = z[i]
        countg = countg +1
        
    if(countg == 0):
      gx = self.green2[3]*2 - self.green2[0]
      gz = (self.green2[4]*2 - self.green2[1] + self.green2[4])/2
      self.green2[0] = self.green2[3]
      self.green2[1] = self.green2[4]
      self.green2[3] = gx
      self.green2[4] = gz
    print(countg)
    #print(countr)
  def verify(self,circles):
    
    y = []
    z = []
    r = []
    mr = []
    mr2 = []
    mg = []
    mg2 = []
    countr = 0
    
    countg = 0
    
    a, b, c = circles.shape
    for i in range(b):
      
      y.append(circles[0,i,0])
      z.append(circles[0,i,1])
      r.append(circles[0,i,2])
    for i in range(b):
      mr.append((y[i] - self.red[3])**2 + (z[i] - self.red[4])**2 + (r[i] - self.red[2])**2)
    for i in range(b):
      mg.append((y[i] - self.green[3])**2 + (z[i] - self.green[4])**2 + (r[i] - self.green[2])**2)  
    for i in range(b):
      if(r[i]*0.7 < self.red[2] and self.red[2] < r[i] *1.3 and mr[i] < 100 and mr[i] == np.min(mr) and countr == 0 or (r[i]*0.9 < self.red[2] and self.red[2] < r[i] * 1.1 and countr == 0 and mr[i] == np.min(mr))):
        self.red[0] = self.red[3]
        self.red[1] = self.red[4]
        self.red[3] = y[i]
        self.red[4] = z[i]
        countr = countr +1
    if(countr == 0):
      ry = self.red[3]*2 - self.red[0]
      rz = (self.red[4]*2 - self.red[1] + self.red2[4])/2
      self.red[0] = self.red[3]
      self.red[1] = self.red[4]
      self.red[3] = ry
      self.red[4] = rz
    #print(self.red)  
    #print(countr)    
    for i in range(b):
      if(r[i]*0.7 < self.green[2] and self.green[2] < r[i] *1.3 and mg[i] < 100 and mg[i] == np.min(mg) and countg == 0 or (r[i]*0.9 < self.green[2] and self.green[2] < r[i] * 1.1 and countg == 0 and mg[i] == np.min(mg))):
        self.green[0] = self.green[3]
        self.green[1] = self.green[4]
        self.green[3] = y[i]
        self.green[4] = z[i]
        countg = countg +1
    if(countg == 0):
      gy = self.green[3]*2 - self.green[0]
      gz = (self.green[4]*2 - self.green[1] + self.green2[4])/2
      self.green[0] = self.green[3]
      self.green[1] = self.green[4]
      self.green[3] = gy
      self.green[4] = gz
    print(countg)
    print(countr)
   # Calculate the conversion from pixel to meter
  def pixel2meter(self):
    circle1Pos = np.array([self.yellow2[3],self.yellow[3],self.yellow[4]])
    circle2Pos = np.array([self.blue2[3],self.blue[3],self.blue[4]])
      # find the distance between two circles
    dist = np.sum((circle1Pos - circle2Pos)**2)
    q = np.sqrt(np.sum((circle1Pos - circle2Pos)**2))
    
    #print(2.5 / np.sqrt(dist))
    return 2.5 / np.sqrt(dist)
    
  # Calculate the relevant joint angles from the image

  def detect_joint_pos(self):
    a = self.pixel2meter()
    # Obtain the centre of each coloured blob
    center = a * np.array([self.yellow2[3],self.yellow[3],(self.yellow[4] + self.yellow2[4])/2])
    circle1Pos1 = a * np.array([self.blue2[3],self.blue[3],(self.blue[4] + self.yellow2[4])/2])
    circle2Pos1 = a * np.array([self.green2[3],self.green[3],(self.green[4]+self.green2[4])/2])
    circle3Pos1 = a * np.array([self.red2[3],self.red[3],(self.red[4]+self.red2[4])/2])
    
    # transfer the x,y cordinate respect to center[0,0] yellow
    circle1Pos = [circle1Pos1[0] - center[0], circle1Pos1[1] - center[1],center[2] - circle1Pos1[2]]
    circle2Pos = [circle2Pos1[0] - center[0], circle2Pos1[1] - center[1],center[2] - circle2Pos1[2]]
    circle3Pos = [circle3Pos1[0] - center[0], circle3Pos1[1] - center[1],center[2] - circle3Pos1[2]]
    
    
    print(circle1Pos+circle2Pos+circle3Pos)
    return np.array(circle1Pos+circle2Pos+circle3Pos)

    


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
