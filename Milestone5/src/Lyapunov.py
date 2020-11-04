#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
import sympy as sym
from sympy import *
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Lyapunov_Pos_Control', anonymous=True) #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Identify the publisher "pub1" to publish on topic "/cmd_vel" to send message of type "Twist"
vel_msg = Twist() #Identify msg variable of data type Twist
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
#######################################################################

#######################################################################
#ROS Subscriber Code for Position
flag_cont_1 = 0	#Initialize flag by zero
pos_des_msg = Pose()	#Identify msg variable of data type Pose
position_des = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback_Des_Pos(data):
  global pos_des_msg	#Identify msg variable created as global variable
  global sub0		#Identify a subscriber as global variable
  global flag_cont_1
  global position_des 

  msg = data
  pos_msg.position.x = round(msg.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg.position.y = round(msg.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg.position.z = round(msg.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg.orientation.x = round(msg.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.y = round(msg.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.z = round(msg.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.w = round(msg.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position_des = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  flag_cont_1 = 1

sub0 = rospy.Subscriber('/APF_Des_Pos', Pose, callback_Des_Pos) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################

#######################################################################
#ROS Subscriber Code for Position
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback_Act_Pos(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub1		#Identify a subscriber as global variable
  global flag_cont
  global position 

  msg = data
  pos_msg.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  flag_cont = 1

sub1 = rospy.Subscriber('/odom', Odometry, callback_Act_Pos) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################

#######################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	#Identify msg variable of data type Pose
position_0 = np.zeros((1,6))
flag_initial = 0
#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Act_Pos_Init(data):
  global pos_msg_0		#Identify msg variable created as global variable
  global sub2			#Identify a subscriber as global variable
  global flag_initial 	#Identify flag created as global variable
  global position_0 

  msg = data
  pos_msg_0.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  pos_msg_0.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  pos_msg_0.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
  pos_msg_0.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
  pos_msg_0.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
  [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
  position_0 = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
  flag_initial = 1
  sub2.unregister()				#Unsubsribe from this topic

sub2 = rospy.Subscriber('/odom', Odometry, callback_Act_Pos_Init) #Identify the subscriber "sub1" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
  pass
#######################################################################

#######################################################################
#Define the initial pose and velocity of the vehicle
Rob_pos_0 = [position_0[0],position_0[1],position_0[3]]
#######################################################################
#########################################################################################################

#########################################################################################################
#Lyapunov Control (Position Control)
## For details about the Lyapunov control, you can chech the textbook (S. G. Tzafestas, Introduction to mobile robot control. Elsevier, 2013.)
def Lyapunov_Control(Rob_pos,Rob_pos_des):
  #Lyapunov Parameters
  K = [rospy.get_param("~K_x"),rospy.get_param("~K_theta")] #Lyapunov Control Gains [K_x,K_theta]
  Cont_input_des = [rospy.get_param("~Vd_des"),rospy.get_param("~Omega_des")] #Desired Control Inputs [Vd_des,Omega_des] 
  
  #Global to Local Transformation 
  err_g = np.array([[Rob_pos_des[0] - Rob_pos[0]],[Rob_pos_des[1] - Rob_pos[1]],[Rob_pos_des[2] - Rob_pos[2]]]) #Evaluate the global error vector [x_G_err,y_G_err,theta_G_err]
  Trans = np.array([[cos(Rob_pos[2]),sin(Rob_pos[2]),0],[-sin(Rob_pos[2]),cos(Rob_pos[2]),0],[0,0,1]]) #Transformation Matrix from global to local axes
  err_l = np.dot(Trans,err_g) #Evaluate the local error vector [x_l_err,y_l_err,theta_l_err]
  
  #Evaluate Control Law
  linear_v = K[0]*err_l[0][0] + Cont_input_des[0]*cos(err_l[2][0]) #Linear Velocity
  angular_v = K[1]*sin(err_l[2][0]) + Cont_input_des[0]*err_l[1][0] +Cont_input_des[1] #Angular Velocity
  Cont_input = [linear_v,angular_v]
  return Cont_input
#########################################################################################################

#########################################################################################################
#Simulation While Loop

while 1 and not rospy.is_shutdown():
    if flag_cont == 1 and flag_cont_1 == 1:
	#Get Robot Current Position and Velocity
	Rob_pos = [position[0],position[1],position[3]]
	Rob_pos_des = [position_des[0],position_des[1],position_des[3]]

	#Calculate the control effort from the Lyaponov-based position control law	
	Cont_input = Lyapunov_Control(Rob_pos,Rob_pos_des)

        flag_cont = 0
	flag_cont_1 = 0
    else:
        #Set the values of the Twist msg to be publeshed
        Cont_input = [0,0]

    v = round(float(Cont_input[0]),2) 	#Linear Velocity	
    w = round(float(Cont_input[1]),2)	#Angular Velocity
    #Set the values of the Twist msg to be publeshed
    vel_msg.linear.x = v #Linear Velocity
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = w #Angular Velocity
    pub1.publish(vel_msg)	#Publish msg
    rate.sleep()		#Sleep with rate
#########################################################################################################
