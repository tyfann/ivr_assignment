#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray, Float64MultiArray, Float64



class robot_control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('robot_control', anonymous=True)

    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # subscriber for joint angles
    self.joints_sub = rospy.Subscriber("joints",Float64MultiArray,self.callback)
     # subscriber for target object position
    self.target_sub = rospy.Subscriber("target_pos",Float64MultiArray,self.targetPosCallback)
    
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')  

    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
    self.target_position = np.array([0.0,0.0,0.0])
    self.end_effector_position = np.array([0.0,0.0,0.0,0.0])
    # print("finished init")


  def targetPosCallback(self,data):
    self.target_position = np.asarray(data.data)

  # Calculate the forward kinematics
  def forward_kinematics(self,joint):
    a=0
    b,c,d = joint
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(-np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    return np.array([x,y,z])


  def calculate_jacobian(self,angle):
    #11
    j1,j2,j3,j4 = angle

    j11 = np.cos(j1)*(np.sin(j2)*np.cos(j3)*(3*np.cos(j4)+3.5)+3*np.cos(j2)*np.sin(j4)) - np.sin(j1)*np.sin(j3)*(3*np.cos(j4) + 3.5)
    j12 = np.sin(j1)*np.cos(j2)*np.cos(j3) * (3*np.cos(j4)+3.5) - 3*np.sin(j1)*np.sin(j2)*np.sin(j4)
    j13 = -np.sin(j1)*np.sin(j2)*np.sin(j3) * (3*np.cos(j4)+3.5) + np.cos(j1)*np.cos(j3) * (3*np.cos(j4) +3.5)
    j14 = -3*np.sin(j1)*np.sin(j2)*np.cos(j3)*np.sin(j4) + 3*np.sin(j1)*np.cos(j2)*np.cos(j4) - 3*np.cos(j1)*np.sin(j3)*np.sin(j4)

    #21
    j21 = np.sin(j1)*(np.sin(j2)*np.cos(j3)*(3*np.cos(j4)+3.5) + 3*np.cos(j2)*np.sin(j4)) + np.cos(j1)*np.sin(j3) * (3*np.cos(j4) +3.5)
    j22 = -np.cos(j1)*np.cos(j2)*np.cos(j3) * (3*np.cos(j4)+3.5) + 3*np.cos(j1)*np.sin(j2)*np.sin(j4)
    j23 = np.cos(1)*np.sin(j2)*np.sin(j3)*(3*np.cos(j4)+3.5) + np.sin(j1)*np.cos(j3)*(3*np.cos(j4)+3.5)
    j24 = 3*np.cos(j1)*np.sin(j2)*np.cos(j3)*np.sin(j4) - 3*np.cos(j1)*np.cos(j2)*np.cos(j4) - 3*np.sin(j1)*np.sin(j3)*np.sin(j4)
    
    #31 ok
    j31 = 0
    j32 = -np.sin(j2)*np.cos(j3) * (3*np.cos(j4)+3.5) -3*np.cos(j2)*np.sin(j4)
    j33 = -np.cos(j2)*np.sin(j3) * (3*np.cos(j4)+3.5)
    j34 = -3*np.cos(j2)*np.cos(j3)*np.sin(j4) - 3*np.sin(j2)*np.cos(j4)

    jacobian_matrix = np.array([[j11,j12,j13,j14],[j21,j22,j23,j24],[j31,j32,j33,j34]])
    
    return jacobian_matrix

  # Closed control of the joints
  def control_closed(self,angle):
    # P gain
    K_p = np.array([[7,0,0],[0,7,0],[0,0,7]])
    # D gain
    K_d = np.array([[0.2,0,0],[0,0.2,0],[0,0,0.2]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.forward_kinematics(angle)
    # desired position
    pos_d= self.target_position 
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = np.zeros(4)
    q[1:4] = angle
    J_inv = np.linalg.pinv(self.calculate_jacobian(q))  # calculating the psudeo inverse of Jacobian
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d

  def callback(self,data):

    q_d = self.control_closed(data.data)
    #q_d = self.control_closed(data.position)
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    # Publish the results
    try: 
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)


def main(args):
  rc = robot_control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

