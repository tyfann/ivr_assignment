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
    self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size = 1)
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

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)


def move():
  rospy.init_node('target_pos_cmd', anonymous=True)
  rate = rospy.Rate(30)  # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
  robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
  t0 = rospy.get_time()
  while not rospy.is_shutdown():
    cur_time = np.array([rospy.get_time()]) - t0
    # y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    x_d = (0.5* np.pi) * np.sin(cur_time * np.pi / 15)
    y_d = (0.5* np.pi) * np.sin(cur_time * np.pi / 18)
    z_d = (0.5* np.pi) * np.sin(cur_time * np.pi / 20)
    joint2 = Float64()
    joint2.data = x_d
    joint3 = Float64()
    joint3.data = y_d
    joint4 = Float64()
    joint4.data = z_d
    robot_joint2_pub.publish(joint2)
    robot_joint3_pub.publish(joint3)
    robot_joint4_pub.publish(joint4)
    rate.sleep()


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
  # try:
  #   move()
  # except rospy.ROSInterruptException:
  #   pass
  main(sys.argv)




