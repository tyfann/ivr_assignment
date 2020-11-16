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
    # initialize a publisher to send messages to a topic named image_topic
    self.joint_sub1 = rospy.Subscriber('joint_pos1', Float64MultiArray, self.callback1)
    self.joint_sub2 = rospy.Subscriber('joint_pos2', Float64MultiArray, self.callback2)

    self.bridge = CvBridge()
    self.camera1_data = None
    self.camera2_data = None



  def callback3(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    


    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))

      self.pos_pub1.publish(self.joints_pos)
      
    except CvBridgeError as e:
      print(e)



  def callback1(self, pos):
      self.camera1_data = np.reshape(np.array(pos.data), [4, 2])
  def callback2(self, pos):
      self.camera2_data = np.reshape(np.array(pos.data), [4, 2])



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