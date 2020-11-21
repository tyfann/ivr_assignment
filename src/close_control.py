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

    # subscriber for joint angles
    self.joints_sub = rospy.Subscriber("target_pos",Float64MultiArray,self.get_joints)

    # subscriber for target object position
    self.target_sub = rospy.Subscriber("target_pos",Int16MultiArray,self.get_target)
    
    # subscriber for end_effector position
    self.end_effector_sub = rospy.Subscriber("/end_pos",Int16MultiArray,self.get_end_effector)

    # to start callback
    self.end_effector_sub2 = rospy.Subscriber("/end_pos",Int16MultiArray,self.callback)
    
    # initialize a publisher to send predicted robot end-effector position
    self.end_effector_pub = rospy.Publisher("/end_pred",Float64MultiArray, queue_size=10)
    
    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    self.epx_pub = rospy.Publisher("/end_predx", Float64, queue_size=10)
    self.epy_pub = rospy.Publisher("/end_predy", Float64, queue_size=10)
    self.epz_pub = rospy.Publisher("/end_predz", Float64, queue_size=10)
    self.eppx_pub = rospy.Publisher("/end_posx", Float64, queue_size=10)
    self.eppy_pub = rospy.Publisher("/end_posy", Float64, queue_size=10)
    self.eppz_pub = rospy.Publisher("/end_posz", Float64, queue_size=10)

    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64') 

    # initialize error and derivative of error for trajectory tracking  
    self.error = np.array([0.0,0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
    print("finished init")

  # Getter for the subscribing data
  def get_joints(self,joints): 
    #print("get joints")
    self.joints = np.array(joints.data)
  def get_target(self,target): 
    #print("get target")
    self.target = np.array(target.data)
  def get_end_effector(self,end_effector): 
    #print("get end_pos")
    self.end_pos = np.array(end_effector.data)

  # Calculate the forward kinematics
  def forward_kinematics(self,joint):

    a,b,c,d = joint
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(-np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    return np.array([x,y,z])

  def jacobian(self):
    #11
    j11 = np.cos(j1)*(np.sin(j2)*np.cos(j3)*(3*np.cos(j4)+3.5)+3*np.cos(j2)*np.sin(j4)) - np.sin(j1)*np.sin(j3)*(3*np.cos(j4) + 3.5)
    j12 = np.sin(j1)*np.cos(j2)*np.cos(j3) * (3*np.cos(j4)+3.5) - 3*np.sin(j1)*np.sin(j2)*np.sin(j4)
    j13 = -np.sin(j1)*np.sin(j2)*np.sin(j3) * (3*np.cos(j4)+3.5) + np.cos(j1)*np.cos(j3) * (3*np.cos(j4) +3.5)
    j14 = -3*np.sin(j1)*np.sin(j2)*np.cos(j3)*np.sin(j4) + 3*np.sin(j1)*np.cos(j2)*np.cos.(j4) - 3*np.cos(j1)*np.sin(j3)*np.sin(j4)

    #21
    j21 = -np.sin(j1)*(-np.sin(j2)*np.cos(j3)*(3*np.cos(j4)+3.5) - 3*np.cos(j2)*np.sin(j4)) + np.cos(j1)*np.sin(j3) * (3*np.cos(j4) +3.5)
    j22 = -np.cos(j1)*np.cos(j2)*np.cos(j3) * (3*np.cos(j4)+3.5) + 3*np.cos(j1)*np.sin(j2)*np.sin(j4)
    j23 = np.cos(1)*np.sin(j2)*np.sin(j3)*(3*np.cos(j4)+3.5) + np.sin(j1)*np.cos(j3)*(3*np.cos(j4)+3.5)
    j24 = 3*np.cos(j1)*np.sin(j2)*np.cos(j3)*np.sin(j4) - 3*np.cos(j1)*np.cos(j2)*np.cos(j4) - 3*np.sin(j1)*np.sin(j3)*np.sin(j4)
    
    #31
    j31 = 0
    j32 = -np.sin(j2)*np.cos(j3) * (3*np.cos(j4)+3.5) -3*np.cos(j2)*np.sin(j4)
    j33 = -np.cos(j2)*np.sin(j3) * (3*np.cos(j4)+3.5)
    j34 = -3*np.cos(j2)*np.cos(j3)*np.sin(j4) - 3*np.sin(j2)*np.sin(j4)
    
    return np.array([[j11,j12,j13,j14],[j21,j22,j23,j24],[j31,j32,j33,j34]])

  # Closed control of the joints
  def control_closed(self,end_effector,target,joints):
    # P gain
    K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
    # D gain
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    # robot end-effector position
    pos = self.detect_end_effector(image)
    # desired trajectory
    pos_d= self.trajectory() 
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos
    q = self.detect_joint_angles(image) # estimate initial value of joints'
    J_inv = np.linalg.pinv(self.calculate_jacobian(image))  # calculating the psudeo inverse of Jacobian
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d


  def callback(self,data):
    #print("start callback")
    # compare the estimated position of robot end-effector calculated by forward kinematics and image
    x_e = self.forward_kinematics(self.joints)
    x_e_image = self.end_pos
    print(x_e , x_e_image*0.04)
    self.end_effector=Float64MultiArray()
    self.end_effector.data= x_e

"""
    # send control commands to joints
    joints = self.joints
    target = self.target
    end_eff = self.end_pos
  
    q_d = self.control_closed(end_eff,target,joints)

    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time

    self.joint1=Float64()
    #self.joint1.data= q_d[0]
    self.joint1.data = 0
    self.joint2=Float64()
    #self.joint2.data= q_d[1]
    self.joint2.data = np.pi/2*np.sin(np.pi/15*rospy.get_time())
    self.joint3=Float64()
    #self.joint3.data= q_d[2]
    self.joint3.data = np.pi/2*np.sin(np.pi/18*rospy.get_time())
    self.joint4=Float64()
    #self.joint4.data= q_d[3]
    self.joint4.data = np.pi/2*np.sin(np.pi/20*rospy.get_time())

    # Publish the results
    self.end_effector_pub.publish(self.end_effector)
    self.robot_joint1_pub.publish(self.joint1)
    self.robot_joint2_pub.publish(self.joint2)
    self.robot_joint3_pub.publish(self.joint3)
    self.robot_joint4_pub.publish(self.joint4)
    self.eppx_pub.publish(x_e[0])
    self.eppy_pub.publish(x_e[1])
    self.eppz_pub.publish(x_e[2])
    self.epx_pub.publish(x_e_image[0]*0.04)
    self.epy_pub.publish(x_e_image[1]*0.04)
    self.epz_pub.publish(x_e_image[2]*0.04)
    rate = rospy.Rate(50)
    rate.sleep()
    """

def main(args):
  rc = robot_control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

