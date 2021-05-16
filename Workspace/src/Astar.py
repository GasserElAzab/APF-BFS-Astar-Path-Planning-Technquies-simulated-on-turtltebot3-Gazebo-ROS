#!/usr/bin/env python

from __future__ import division
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from PIL import Image
import random
import cv2
import collections
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
import sympy as sym
from sympy import *



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
rospy.init_node('Path_Planning_BFS', anonymous=True) #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/Astar_Des_Pos', Pose, queue_size=10) #Identify the publisher "pub1" to publish on topic "/APF_Des_Pos" to send message of type "Pose"
Des_Pos_msg = Pose() #Identify msg variable of data type Twist
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################
global distance_delta
global angle_delta
#######################################################################
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]
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

def dist_diff(x, y, x_des, y_des):

    xco = x
    yco = y

    x_delta = x_des - xco		#Calculate the change in X direction
    y_delta = y_des - yco	#Calculate the change in Y direction

    #Calculate distance rho representing relative distance between the desired and the current position
    distance_delta = np.sqrt((np.square(x_delta))+ (np.square(y_delta)))
    #Calculate angle gamma representing the angle between the global X-direction of the vehicle and rho
    angle_delta = np.arctan2(y_delta,x_delta)	
    return distance_delta , angle_delta


#######################################################################
#ROS Subscriber Code for Position
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
Velocity_msg = Twist()
velocity = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub2		#Identify a subscriber as global variable
  global flag_cont
  global position 
  global Velocity_msg
  global velocity

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
  Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]
  flag_cont = 1

sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
#######################################################################

#######################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	#Identify msg variable of data type Pose
position_0 = np.zeros((1,6))
flag_initial = 0
Velocity_msg_0 = Twist()
velocity_0 = np.zeros((1,6))
#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Init(data):
  global pos_msg_0		#Identify msg variable created as global variable
  global sub1			#Identify a subscriber as global variable
  global flag_initial 	#Identify flag created as global variable
  global position_0 
  global Velocity_msg_0
  global velocity_0

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
  Velocity_msg_0.linear.x = round(msg.twist.twist.linear.x, 4)
  Velocity_msg_0.linear.y = round(msg.twist.twist.linear.y, 4)
  Velocity_msg_0.linear.z = round(msg.twist.twist.linear.z, 4)
  Velocity_msg_0.angular.x = round(msg.twist.twist.angular.x, 4)
  Velocity_msg_0.angular.y = round(msg.twist.twist.angular.y, 4)
  Velocity_msg_0.angular.z = round(msg.twist.twist.angular.z, 4)
  velocity_0 = [Velocity_msg_0.linear.x,Velocity_msg_0.linear.y,Velocity_msg_0.linear.z,Velocity_msg_0.angular.x,Velocity_msg_0.angular.y,Velocity_msg_0.angular.z]
  flag_initial = 1
  sub1.unregister()				#Unsubsribe from this topic

sub1 = rospy.Subscriber('/odom', Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
  pass
#######################################################################

#######################################################################
#Define the initial pose and velocity of the vehicle
Rob_pos_0 = [position_0[0],position_0[1],position_0[3]]
Roc_vel_0 = [velocity_0[0],velocity_0[5]]

x_p = Rob_pos_0[0]
y_p = Rob_pos_0[1]
vel_p_x = Roc_vel_0[0]*cos(Rob_pos_0[2])
vel_p_y = Roc_vel_0[0]*sin(Rob_pos_0[2])
#######################################################################
#########################################################################################################

#########################################################################################################
#######################################################################
#APF Inputs
img = cv2.imread('/home/gasser/catkin_ws/src/Milestone5/src/gasser.png')  #Read image 
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #Convert RGB image to grayscale
ret, bw_img = cv2.threshold(grayImage,0,255,cv2.THRESH_BINARY) #Convert grayscale image to binary
bw_img = bw_img.astype(np.uint8)

h, w= bw_img.shape #get image dimenssions
x_Goal= rospy.get_param("~x_Goal")*10
y_Goal= rospy.get_param("~y_Goal")*10
x_Start= 50
y_Start= 5
def Dis_to_Goal():
    global h, w, bw_img, x_Goal, y_Goal
    dx_mat = [0] * bw_img
    dy_mat = [0] * bw_img
    dis_mat = [0] * bw_img
    for i in range(0,h):
	for j in range(0,w): 
  	    dy_mat[i,j] = abs(y_Goal - i)
	    dx_mat[i,j] = abs(x_Goal - j)
    dis_mat = np.sqrt(np.square(dx_mat)+np.square(dy_mat))
    return dis_mat, dx_mat, dy_mat



#########################################################################################################
dis_mat, dx_mat, dy_mat = Dis_to_Goal()
#########################################################################################################


def A_star():
    global x_Start, y_Start, x_Goal, y_Goal, dis_mat, h, w

    path_x = [x_Start]
    path_y = [y_Start]
    x_curr = x_Start
    y_curr = y_Start
    mask_x = np.array([[-1,0,1], [-1,0,1], [-1,0,1]])
    mask_y = np.array([[-1,-1,-1], [0,0,0], [1,1,1]])
    dis_to_goal_list = []
    dis_from_start_list = []
    x_neig_index = []
    y_neig_index = []
    Travelled_distance = [0]	

    

    #for i in range(0,1):
    while any([x_curr != x_Goal,y_curr != y_Goal]) and not rospy.is_shutdown():
	if bw_img[y_curr,x_curr] != 0: 
	        ind_x = mask_x + [x_curr]
                ind_y = mask_y + [y_curr]
	        for k in range(0,3):
		    for l in range(0,3):
		        if all([ind_x[k,l] >= 0,ind_x[k,l] < w,ind_y[k,l] >= 0,ind_y[k,l] < h]):
			    l1 = np.where(path_x == ind_x[k,l])[0]
			    l2 = np.where(path_y == ind_y[k,l])[0]
			    flag = 0
			    for i in range(0,len(l1)):
			        for j in range(0,len(l2)):
				    if l1[i] == l2[j]:
				        flag = 1
			    if  any([all([ind_x[k,l] == x_curr,ind_y[k,l] == y_curr]),flag == 1]):
			        pass
  			    else:
				if bw_img[ind_y[k,l],ind_x[k,l]] != 0:
				    #if neighbour is already in the path
				    x_neig_index = np.concatenate((x_neig_index, mask_x[k,l]), axis=None)
				    y_neig_index = np.concatenate((y_neig_index, mask_y[k,l]), axis=None)
				    dis_to_goal_list = np.concatenate((dis_to_goal_list, dis_mat[ind_y[k,l],ind_x[k,l]]), axis=None)
				    dis_from_start_list = np.concatenate((dis_from_start_list, Travelled_distance+np.sqrt(np.square(mask_x[k,l])+np.square(mask_y[k,l]))), axis=None)
	        #print('x_neighbous', x_neig_index)
	        #print('y_neighbous', y_neig_index)
                #print('dis_to_goal_list', dis_to_goal_list)
	        #print('dis_from_start_list', dis_from_start_list)
	if all([len(x_neig_index) != 0,len(y_neig_index) != 0]): 
		obj_fn = dis_to_goal_list + dis_from_start_list
		#obj_fn = dis_to_goal_list
		result = np.where(obj_fn == np.amin(obj_fn))
		path_x = np.concatenate((path_x, x_curr+x_neig_index[result[0][0]]), axis=None)
		path_y = np.concatenate((path_y, y_curr+y_neig_index[result[0][0]]), axis=None)
		Travelled_distance = dis_from_start_list[result[0][0]]
		x_curr = float(x_curr+x_neig_index[result[0][0]])
		y_curr = float(y_curr+y_neig_index[result[0][0]])
		x_neig_index = []
		y_neig_index = []
		dis_to_goal_list = []
		dis_from_start_list = []
		#print('path_x', path_x)
		#print('path_y', path_y)
		#print('obj_fn', obj_fn)
		#print('-----------------------------')
	else:
		print('This is the path that could be found so Far .. The algorithm is uncomplete!!!clear')
		break
    return path_x, path_y


path_x, path_y = A_star()
count=0
x_des=path_x[0]
y_des=path_y[0]



while 1 and not rospy.is_shutdown() and count<(len(path_x)):
    #Get Robot Current Position and Velocity
	if flag_cont == 1:
		Rob_pos = [position[0],position[1],position[3]]
		distance_delta, angle_delta = dist_diff(Rob_pos[0], Rob_pos[1] , x_des, y_des)
		if distance_delta <= 0.45 and count<(len(path_y)-1): 
			count = count+1
		Rob_pos = [position[0],position[1],position[3]]
		Rob_vel = [velocity[0],velocity[5]]
		x_des=path_x[count]/10
		y_des=path_y[count]/10
	 	flag_cont=0
	#Calculate the desired robot position from the APF

	#Update the previous robot states for the next iteration

	else:
		x_des=path_x[count]/10
		y_des=path_y[count]/10

    
	Des_Pos_msg.position.x = round(x_des,4)
	Des_Pos_msg.position.y = round(y_des, 4)
	Des_Pos_msg.position.z = 0
	[qx_des, qy_des, qz_des, qw_des] = euler_to_quaternion(angle_delta, 0, 0)
	Des_Pos_msg.orientation.x = round(qx_des,4)
	Des_Pos_msg.orientation.y = round(qy_des,4)
	Des_Pos_msg.orientation.z = round(qz_des,4)
	Des_Pos_msg.orientation.w = round(qw_des,4)
	print ("was at")
	print(position[0], position[1])
	print("3aiz atnyl")
	print(x_des, y_des)

	pub1.publish(Des_Pos_msg)	#Publish msg
	rate.sleep()		#Sleep with rate
#########################################################################################################




