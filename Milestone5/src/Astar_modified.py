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
rospy.init_node('Path_Planning_Astar2', anonymous=True) #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/Astar2_Des_Pos', Pose, queue_size=10) #Identify the publisher "pub1" to publish on topic "/APF_Des_Pos" to send message of type "Pose"
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
class Node:
    def __init__(self, value, point):
        self.value = value
        self.point = point
        self.parent = None
        self.H = 0
        self.G = 0

    def move_cost(self, other):
        return 0 if self.value == '.' else 1

def initialize_twodlist(foo):
    twod_list = []
    new = []
    for i in range (0, 100):
        for j in range (0, 40):
            new.append(foo)
        twod_list.append(new)
        new = []
    return twod_list


def children(point, grid):
    x, y = point.point
    links=[]
    if (x - 1 > 0 and grid[x - 1][y].value != 0):
        links.append(grid[x - 1][y])
    if (y - 1 > 0 and grid[x][y - 1].value != 0):
        links.append(grid[x][y - 1])
    if (x + 1 < 100 and grid[x + 1][y].value != 0):
        links.append(grid[x + 1][y])
    if (y + 1 < 40 and grid[x][y + 1].value != 0):
        links.append(grid[x][y + 1])
    if (x - 1 > 0 and y - 1 > 0 and grid[x - 1][y - 1].value != 0):
        links.append(grid[x - 1][y - 1])
    if (x + 1 < 100 and y + 1 < 40 and grid[x + 1][y + 1].value != 0):
        links.append(grid[x + 1][y + 1])
    if (x + 1 < 100 and y - 1 > -1 and grid[x + 1][y - 1].value != 0):
        links.append(grid[x + 1][y - 1])
    if (x - 1 > 0 and y + 1 < 100 and grid[x - 1][y + 1].value != 0):
        links.append(grid[x - 1][y + 1])
    return links


def manhattan(point, point2):
    xd=abs(point.point[0] - point2.point[0])
    yd=abs(point.point[1] - point2.point[1])
    dis= np.sqrt(np.square(xd)+np.square(yd))
    return dis


def aStar(start, goal, grid):
    # The open and closed sets
    openset = set()
    closedset = set()
    # Current point is the starting point
    current = start
    # Add the starting point to the open set
    openset.add(current)
    # While the open set is not empty
    q=0
    while openset:
        q=q+1
        print(q)
        # Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o: o.G + o.H)
        # If it is the item we want, retrace the path and return it
        if current == goal:
            path = []

            while current.parent:
                path.append(current)

                current = current.parent
            path.append(current)


            return path[::-1]
        # Remove the item from the open set
        openset.remove(current)
        # Add it to the closed set
        closedset.add(current)
        # Loop through the node's children/siblings
        links=children(current, grid)
        for node in links :
            # If it is already in the closed set, skip it

            if node in closedset:
                continue
            # Otherwise if it is already in the open set
            if node in openset:
                # Check if we beat the G score
                new_g = current.G + current.move_cost(node)
                if node.G > new_g:
                    # If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = manhattan(node, goal)
                # Set the parent to our current item
                node.parent = current
                # Add it to the set
                openset.add(node)
    # Throw an exception if there is no path

    raise ValueError('No Path Found')


def next_move(pacman, food, gamed,grid):
    # Convert all the points to instances of Node
    for x in range(len(grid)):
        for y in range(len(grid[x])):
            grid[x][y] = Node(gamed[x,y], (x, y))
    # Get the path

    path = aStar(grid[pacman[0]][pacman[1]], grid[food[0]][food[1]], grid)
    # Output the path
    len(path) - 1
    p=[]
    t=0
    while t < len(path):
        a=path[t].point[0]
        b=path[t].point[1]
        p.append([a,b])
        t=t+1
    return p

width = 40;
height= 100;
img = Image.new ('1', (height, width)) #array = np.zeros([height, width], dtype=np.uint)
pixels= img.load()
for i in range(img.size[0]):
    for j in range(img.size[1]):
        pixels[i,j] = 255;
        if j==27 or j==28 or j==29 or j==30 or j==31 or j==32 or j==33:
            if i>3 and i<17 or i>21 and i<33 or i==50 or i>57 and i<73 or i== 80 or i==90 or i==95:
                pixels[i,j] = 0;
        if j==18 or j==19 or j==20 or j==21 or j==22:
            if i>3 and i<17 or i==30 or i>33 and i<42 or i==50 or i>57 and i<73 or i>78 and i<92 or i==95:
                pixels[i,j] = 0;
        if j==8 or j==9 or j==10 or j==11 or j==12:
            if i>3 and i<17 or i==30 or i>57 and i<73 or i==90 or i==95 or i==50:
                pixels[i,j] = 0;

        if (i==13 or i==14 or i==15 or i==16 or i==17 or i == 12 or i == 11) and j>=8 and j<=33:
                pixels[i,j] = 0;

        if (i== 28 or i==29 or i==30 or i==31 or i==32) and j>=8 and j<=33:
                pixels[i,j] = 0;

        if (i==48 or i==49 or i==50 or i==51 or i==52) and j>=8 and j<=33:
                pixels[i,j] = 0;

        if (i== 58or i==59 or i== 60 or i==61 or i==62) and j>=18 and j<=33:
                pixels[i,j] = 0;

        if (i==68 or i== 69 or i== 70 or i==71 or i== 72) and j>=8 and j<=22:
                pixels[i,j] = 0;

        if (i==78 or i==79 or i==80 or i==81 or i==82) and j>=18 and j<=33:
                pixels[i,j] = 0;

        if (i==89 or i==90 or i== 91 ) and j>=8 and j<=33:
                pixels[i,j] = 0;

        if (i== 94 or i==95 or i== 96 ) and j>=8 and j<=33:
                pixels[i,j] = 0;
        if(i<100) and j==0 or j==1 or j==2 or j==37 or j== 38 or j==39:
            pixels[i, j] = 0;
        if (j < 40) and i == 0 or i == 1 or i == 2 or i == 98 or i == 99:
            pixels[i, j] = 0;

global grid
grid=initialize_twodlist(0)
pacman_x, pacman_y = [5,5]
food_x, food_y = [35,35]
x=100
y=40
a=0
meshwar=next_move([pacman_x, pacman_y], [food_x, food_y], pixels,grid)
count=0
x_des=(meshwar[0])[0]
y_des= (meshwar[0])[1]



while 1 and not rospy.is_shutdown() and count<(len(meshwar)):
    #Get Robot Current Position and Velocity
	if flag_cont == 1:
		Rob_pos = [position[0],position[1],position[3]]
		distance_delta, angle_delta = dist_diff(Rob_pos[0], Rob_pos[1] , x_des, y_des)
		if distance_delta <= 0.05 and count<(len(meshwar)-1): 
			count = count+1
		Rob_pos = [position[0],position[1],position[3]]
		Rob_vel = [velocity[0],velocity[5]]
		x_des=(meshwar[count])[0]/10
		y_des=(meshwar[count])[1]/10
	 	flag_cont=0
	#Calculate the desired robot position from the APF

	#Update the previous robot states for the next iteration

	else:
		x_des=(meshwar[count])[0]/10
		y_des=(meshwar[count])[0]/10

    
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


