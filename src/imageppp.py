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
import math as m

class image_converter:
                                                                                                                                                                               
  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send messages to a topic named image_topic
    self.camera1_data = None
    self.camera2_data = None
    self.storageR1 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    self.storageB1 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    self.storageG1 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    self.storageT1 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    
    
    self.storageR2 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    self.storageB2 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    self.storageG2 = np.array([0.0,0.0,0.0,0.0],dtype='float64')
    self.storageT2 = np.array([0.0,0.0,0.0,0.0],dtype='float64')

    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')
    self.x = None  
    self.y = None
    self.x2 = None
    self.z = None
  
    self.joint_sub1 = rospy.Subscriber('joint_pos1', Float64MultiArray, self.callback1)
    
    self.joint_sub2 = rospy.Subscriber('joint_pos2', Float64MultiArray, self.callback2)


    self.joints_pub = rospy.Publisher('joints',Float64MultiArray,queue_size = 10)
    self.target_pub = rospy.Publisher('target_pos',Float64MultiArray,queue_size = 10)
    self.bridge = CvBridge()
    



  def callback1(self, pos):
      
      self.camera1_data = np.reshape(np.array(pos.data), [4, 2])
      self.pos1()
      
       
      
  def callback2(self, pos):

      self.greenj = Float64MultiArray()
      self.redj = Float64MultiArray()
      self.time_previous_step = self.time_trajectory
      self.time_trajectory = rospy.get_time()
      self.camera2_data = np.reshape(np.array(pos.data), [4, 2])
      self.pos2()
      if((self.camera2_data is not None) and (self.camera1_data is not None)):
        joint = Float64MultiArray()
        joint.data = self.detect_joint_angles()
        #self.rotation()
        #self.rot2()
        #self.greenj.data = self.calculate_green()
        #self.redj.data = self.calculate_red()
        target = Float64MultiArray()
        target.data = [self.camera2_data[3,0],self.camera1_data[3,0],(self.camera1_data[3,1]+self.camera2_data[3,1])/2]
        try:
          self.joints_pub.publish(joint)
          self.target_pub.publish(target)
        except CvBridgeError as e:
            print(e)
        

  def rotation(self):
    self.z = m.pi*np.sin((m.pi/15)*(self.time_trajectory - self.time_previous_step))
    self.x = 0.5*m.pi*np.sin((m.pi/15)*(self.time_trajectory - self.time_previous_step))
    self.y = 0.5*m.pi*np.sin((m.pi/18)*(self.time_trajectory - self.time_previous_step))
    self.x2 = 0.5*m.pi*np.sin((m.pi/20)*(self.time_trajectory - self.time_previous_step))
    return self
  
  def rot2(self):
    a = self.Rx().dot(self.Ry()).dot(self.Rx2())
    print(a)
    return a
  def rot4(self):
    return self.Rz().dot(self.Rx().dot(self.Ry()).dot(self.Rx2()))  
    
  def Rx(self):
    return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(self.x),-m.sin(self.x)],
                   [ 0, m.sin(self.x), m.cos(self.x)]])
  def Rx2(self):
    return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(self.x2),-m.sin(self.x2)],
                   [ 0, m.sin(self.x2), m.cos(self.x2)]])
  
  def Ry(self):
    return np.matrix([[ m.cos(self.y), 0, m.sin(self.y)],
                   [ 0           , 1, 0           ],
                   [-m.sin(self.y), 0, m.cos(self.y)]])
  
  def Rz(self):
    return np.matrix([[ m.cos(self.z), -m.sin(self.z), 0 ],
                   [ m.sin(self.z), m.cos(self.z) , 0 ],
                   [ 0           , 0            , 1 ]])
    

  def pos1(self):
    if(self.storageB1[2] == 0.0 and (self.storageB1[0] != 0.0 )):
      self.storageB1[2] =  self.camera1_data[0,0]
      self.storageB1[3] =  self.camera1_data[0,1]
    
    if(self.storageB1[0] == 0.0 and(self.storageB1[2] == 0.0)):
      self.storageB1[0] =  self.camera1_data[0,0]
      self.storageB1[1] =  self.camera1_data[0,1]
      
    if(self.storageG1[2] == 0.0 and (self.storageG1[0] != 0.0 )):
      self.storageG1[2] =  self.camera1_data[1,0]
      self.storageG1[3] =  self.camera1_data[1,1]  

    if(self.storageG1[0] == 0.0 and(self.storageG1[2] == 0.0)):
      self.storageG1[0] =  self.camera1_data[1,0]
      self.storageG1[1] =  self.camera1_data[1,1]
      
    if(self.storageR1[2] == 0.0 and (self.storageR1[0] != 0.0 )):
      self.storageR1[2] =  self.camera1_data[2,0]
      self.storageR1[3] =  self.camera1_data[2,1]

    if(self.storageR1[0] == 0.0 and(self.storageR1[2] == 0.0)):
      self.storageR1[0] =  self.camera1_data[2,0]
      self.storageR1[1] =  self.camera1_data[2,1]
      
    if(self.storageT1[2] == 0.0 and (self.storageT1[0] != 0.0 )):
      self.storageT1[2] =  self.camera1_data[3,0]
      self.storageT1[3] =  self.camera1_data[3,1]        

    if(self.storageT1[0] == 0.0 and(self.storageT1[2] == 0.0)):
      self.storageT1[0] =  self.camera1_data[3,0]
      self.storageT1[1] =  self.camera1_data[3,1]     

    if((self.storageT1[2] != 0.0) and (self.storageB1[2] != 0.0) and(self.storageG1[2] != 0.0) and(self.storageR1[2] != 0.0)):    
      if(self.camera1_data[0,0] == 0):
          self.camera1_data[0,0] = self.storageB1[2]*2 - self.storageB1[0]
          self.camera1_data[0,1] = ((self.storageB1[3]*2 - self.storageB1[1]) + self.camera2_data[0,1])/2
          self.storageB1[0] = self.storageB1[3]
          self.storageB1[1] = self.storageB1[2]
          self.storageB1[2] = self.camera1_data[0,0]
          self.storageB1[3] = self.camera1_data[0,1]
      else:
          self.storageB1[0] = self.storageB1[2]
          self.storageB1[1] = self.storageB1[3]
          self.storageB1[2] = self.camera1_data[0,0]
          self.storageB1[3] = self.camera1_data[0,1]

      if(self.camera1_data[1,0] == 0):
          self.camera1_data[1,0] = self.storageG1[2]*2 - self.storageG1[0]
          self.camera1_data[1,1] = ((self.storageG1[3]*2 - self.storageG1[1]) + self.camera2_data[1,1])/2
          self.storageG1[0] = self.storageG1[3]
          self.storageG1[1] = self.storageG1[2]
          self.storageG1[2] = self.camera1_data[1,0]
          self.storageG1[3] = self.camera1_data[1,1]
      else:
          self.storageG1[0] = self.storageG1[2]
          self.storageG1[1] = self.storageG1[3]
          self.storageG1[2] = self.camera1_data[1,0]
          self.storageG1[3] = self.camera1_data[1,1]

      if(self.camera1_data[2,0] == 0):
          self.camera1_data[2,0] = self.storageR1[2]*2 - self.storageR1[0]
          self.camera1_data[2,1] = ((self.storageR1[3]*2 - self.storageR1[1]) + self.camera2_data[2,1])/2
          self.storageR1[0] = self.storageR1[3]
          self.storageR1[1] = self.storageR1[2]
          self.storageR1[2] = self.camera1_data[2,0]
          self.storageR1[3] = self.camera1_data[2,1]
      else:
          self.storageR1[0] = self.storageR1[2]
          self.storageR1[1] = self.storageR1[3]
          self.storageR1[2] = self.camera1_data[2,0]
          self.storageR1[3] = self.camera1_data[2,1]

      if(self.camera1_data[3,0] == 0):
          self.camera1_data[3,0] = self.storageT1[2]*2 - self.storageT1[0]
          self.camera1_data[3,1] = ((self.storageT1[3]*2 - self.storageT1[1]) + self.camera2_data[3,1])/2
          self.storageT1[0] = self.storageT1[3]
          self.storageT1[1] = self.storageT1[2]
          self.storageT1[2] = self.camera1_data[3,0]
          self.storageT1[3] = self.camera1_data[3,1]
      else:
          self.storageT1[0] = self.storageT1[2]
          self.storageT1[1] = self.storageT1[3]
          self.storageT1[2] = self.camera1_data[3,0]
          self.storageT1[3] = self.camera1_data[3,1]
          
    return self.camera1_data    







  def pos2(self):
    if(self.storageB2[0] == 0.0 and(self.storageB2[2] == 0.0)):
      self.storageB2[0] =  self.camera2_data[0,0]
      self.storageB2[1] =  self.camera2_data[0,1]

    if(self.storageB2[2] == 0.0 and (self.storageB2[0] != 0.0 )):
      self.storageB2[2] =  self.camera2_data[0,0]
      self.storageB2[3] =  self.camera2_data[0,1]

    if(self.storageG2[0] == 0.0 and(self.storageG2[2] == 0.0)):
      self.storageG2[0] =  self.camera2_data[1,0]
      self.storageG2[1] =  self.camera2_data[1,1]

    if(self.storageG2[2] == 0.0 and (self.storageG2[0] != 0.0 )):
      self.storageG2[2] =  self.camera2_data[1,0]
      self.storageG2[3] =  self.camera2_data[1,1]  

    if(self.storageR2[0] == 0.0 and(self.storageR2[2] == 0.0)):
      self.storageR2[0] =  self.camera2_data[2,0]
      self.storageR2[1] =  self.camera2_data[2,1]

    if(self.storageR2[2] == 0.0 and (self.storageR2[0] != 0.0 )):
      self.storageR2[2] =  self.camera2_data[2,0]
      self.storageR2[3] =  self.camera2_data[2,1]

    if(self.storageT2[0] == 0.0 and(self.storageT2[2] == 0.0)):
      self.storageT2[0] =  self.camera2_data[3,0]
      self.storageT2[1] =  self.camera2_data[3,1]

    if(self.storageT2[2] == 0.0 and (self.storageT2[0] != 0.0 )):
      self.storageT2[2] =  self.camera2_data[3,0]
      self.storageT2[3] =  self.camera2_data[3,1]       

    if((self.storageT2[2] != 0.0) and (self.storageB2[2] != 0.0) and(self.storageG2[2] != 0.0) and(self.storageR2[2] != 0.0)):    

   
      if(self.camera2_data[0,0] == 0):
          self.camera2_data[0,0] = self.storageB2[2]*2 - self.storageB2[0]
          self.camera2_data[0,1] = ((self.storageB2[3]*2 - self.storageB2[1]) + self.camera1_data[0,1])/2
          self.storageB2[0] = self.storageB2[2]
          self.storageB2[1] = self.storageB2[3]
          self.storageB2[2] = self.camera2_data[0,0]
          self.storageB2[3] = self.camera2_data[0,1]
      else:
          self.storageB2[0] = self.storageB2[2]
          self.storageB2[1] = self.storageB2[3]
          self.storageB2[2] = self.camera2_data[0,0]
          self.storageB2[3] = self.camera2_data[0,1]

      if(self.camera2_data[1,0] == 0):
          self.camera2_data[1,0] = self.storageG2[2]*2 - self.storageG2[0]
          self.camera2_data[1,1] = ((self.storageG2[3]*2 - self.storageG2[1]) + self.camera1_data[1,1])/2
          self.storageG2[0] = self.storageG2[3]
          self.storageG2[1] = self.storageG2[2]
          self.storageG2[2] = self.camera2_data[1,0]
          self.storageG2[3] = self.camera2_data[1,1]
      else:
          self.storageG2[0] = self.storageG2[2]
          self.storageG2[1] = self.storageG2[3]
          self.storageG2[2] = self.camera2_data[1,0]
          self.storageG2[3] = self.camera2_data[1,1]

      if(self.camera2_data[2,0] == 0):
          self.camera2_data[2,0] = self.storageR2[2]*2 - self.storageR2[0]
          self.camera2_data[2,1] = ((self.storageR2[3]*2 - self.storageR2[1]) + self.camera1_data[2,1])/2
          self.storageR2[0] = self.storageR2[3]
          self.storageR2[1] = self.storageR2[2]
          self.storageR2[2] = self.camera2_data[2,0]
          self.storageR2[3] = self.camera2_data[2,1]
      else:
          self.storageR2[0] = self.storageR2[2]
          self.storageR2[1] = self.storageR2[3]
          self.storageR2[2] = self.camera2_data[2,0]
          self.storageR2[3] = self.camera2_data[2,1]

      if(self.camera2_data[3,0] == 0):
          self.camera2_data[3,0] = self.storageT2[2]*2 - self.storageT2[0]
          self.camera2_data[3,1] = ((self.storageT2[3]*2 - self.storageT2[1]) + self.camera1_data[3,1])/2
          self.storageT2[0] = self.storageT2[3]
          self.storageT2[1] = self.storageT2[2]
          self.storageT2[2] = self.camera2_data[3,0]
          self.storageT2[3] = self.camera2_data[3,1]
      else:
          self.storageT2[0] = self.storageT2[2]
          self.storageT2[1] = self.storageT2[3]
          self.storageT2[2] = self.camera2_data[3,0]
          self.storageT2[3] = self.camera2_data[3,1]
    return self.camera2_data   

  def detect_joint_angles(self):
    blue = np.array([self.camera2_data[0,0],self.camera1_data[0,0],(self.camera2_data[0,1]+self.camera1_data[0,1])/2])
    green = ([self.camera2_data[1,0],self.camera1_data[1,0],(self.camera2_data[1,1]+self.camera1_data[1,1])/2])
    red = np.array([self.camera2_data[2,0],self.camera1_data[2,0],(self.camera2_data[2,1]+self.camera1_data[2,1])/2])
    theta2 = 0
    theta3 = 0
    if((green[2]-2.5)==0):
      theta2 = 0
      theta3 = 0
    elif(green[1]==0):
      theta3 = 0
    else:
      theta2 = np.arctan(-green[1]/(green[2]-2.5))
      theta3 = np.arctan(-np.sin(theta2)*green[0]/green[1])
      
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
    
    
    return np.array([theta2,theta3,theta4])
  
       
  

  def calculate_red(self):
      pos_r = np.array([self.camera2_data[2,0],self.camera1_data[2,0],(self.camera2_data[2,1]+self.camera1_data[2,1])/2])
  
      theta2,theta3 = self.calculate_green()
      cos4 = (pos_r[0]/np.sin(theta3)-3.5)/3
      sin4 = (((pos_r[1]/np.sin(theta2))+(pos_r[2]/np.cos(theta2)))-2.5/np.cos(theta2))/(-3*(np.tan(theta2)+1/np.tan(theta2)))
      theta4 = np.arctan(sin4/cos4)
      # print(theta2,theta3,theta4)
      return np.array([theta2,theta3,theta4])
    



 
      
      

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
