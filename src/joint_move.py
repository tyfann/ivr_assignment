#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64


# Publish data
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



# run the code if the node is called
if __name__ == '__main__':
  try:
    move()
  except rospy.ROSInterruptException:
    pass



