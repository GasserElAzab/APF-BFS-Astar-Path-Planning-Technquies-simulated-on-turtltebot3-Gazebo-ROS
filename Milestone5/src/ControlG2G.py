#!/usr/bin/env python
# A basic python code to implement the kinematics model for the differential mobile robot
# and reflect its behavior on graphs.

#########################################################################################################
#Import the required libraries:
import rospy
import math
from geometry_msgs.msg import Twist   #import msg data type "Twist" to be published
from geometry_msgs.msg import PoseWithCovariance   #import msg data type "Twist" to be published
#from turtlesim.msg import Pose        #import msg data type "Pose" to be subscribed
import numpy as np                    #import numpy for trignometric function, arrays... etc
import sys                            #import sys for extracting input from termminal (input from user)
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#########################################################################################################

#########################################################################################################
i=3
x=[]
y=[]
th=[]
while i<6:
 x.append(i)
 y.append(0)
 th.append(0)
 i=i+0.5
i=0
xs=x[-1]
while i<14:
 x.append((xs+2*math.sin(i)))
 y.append(i)
 th.append(np.pi/2)
 i=i+0.5
#########################################################################################################

#########################################################################################################
#Initialize ROS Node
rospy.init_node('Point_to_Point_Control', anonymous=True) #Identify ROS Node
#######################################################################
#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=10) #Identify the publisher "pub1" to publish on topic "/turtle1/cmd_vel" to send message of type "Twist"
vel_msg = Twist() #Identify msg variable of data type Twist
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################
#######################################################################
#ROS Subscriber Code for Position
flag_initial_Pos = 0	#Initialize flag by zero
xcordinit = 0 
ycordinit = 0
thetayawinit = 0
xcord = 0
ycord = 0
thetayaw = 0
#pos_msg_0 = Pose()	#Identify msg variable of data type Pose
#pos_msg = Pose()	#Identify msg variable of data type Pose
#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Init(data):
  global pos_msg_0		#Identify msg variable created as global variable
  global sub1			#Identify a subscriber as global variable
  global flag_initial_Pos 	#Identify flag created as global variable
  flag_initial_Pos = 1		#Set flag to one
  global xcordinit
  global ycordinit
  global yawthetainit

  pos_msg_0 = data				#Initialize pos_msg with data sent to the function
  xcord = data.pose.pose.position.x
  ycord = data.pose.pose.position.y
  #pos_msg_0.x = round(pos_msg_0.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  #pos_msg_0.y = round(pos_msg_0.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  #theta quateroni
  #currentphx.publish(data.pose.pose.orientation.x)
  #currentphy.publish(data.pose.pose.orientation.y)
  #currentphz.publish(data.pose.pose.orientation.z)
  #currentphw.publish(data.pose.pose.orientation.w)
  quaternion = (data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
  euler = euler_from_quaternion(quaternion)
  #roll = euler[0]
  #pitch = euler[1]
  yaw = euler[2]

  thetayawinit = round(yaw, 4)	#Round the value of theta to 4 decimal places
  #rospy.loginfo(pos_msg_0)			#Print initial Pose of the vehicle on terminal
  sub1.unregister()				#Unsubsribe from this topic

sub1 = rospy.Subscriber("robot3/odom", Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub2		#Identify a subscriber as global variable
  global thetayaw
  global xcord
  global ycord
  
  #pos_msg = data				#Initialize pos_msg with data sent to the function
  xcord = round(data.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  ycord = round(data.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  quaternion = (data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
  euler = euler_from_quaternion(quaternion)
  yaw = euler[2]
  thetayaw = round(yaw, 4)	#Round the value of theta to 4 decimal places
  
sub2 = rospy.Subscriber("robot3/odom", Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial_Pos == 0:
	print("ana hena")
	pass
#########################################################################################################

#########################################################################################################
#Define the initial pose of the vehicle: Can get it from /turtle1/pose
x0 = xcordinit
y0 = ycordinit
theta0 = thetayawinit
#######################################################################
#######################################################################
#Initialize the parameters for coordinate transformation
rho = 0 #Initialization of variable rho
beta = 0 #Initialization of variable beta
alpha = 0 #Initialization of variable alpha
#######################################################################
#######################################################################
#Initialize the control gains
Krho = 0.18
Kalpha = 1.2
Kbeta = -0.15
#######################################################################
#######################################################################
#Initialize controller output
linear_v = 0 #Initialize linear velocity 
angular_v = 0 #Initialize angular velocity
#########################################################################################################

#########################################################################################################
#Coordinate transformation function to transform to polar coordinates
def transformation(x, y, yaw):
    global rho		#Identify rho variable as global variable
    global beta		#Identify beta variable as global variable
    global alpha	#Identify alpha variable as global variable
    global x_des
    global y_des
    global theta_des
    xco = x
    yco = y
    yawco = yaw

    x_delta = x_des - xco		#Calculate the change in X direction
    y_delta = y_des - yco	#Calculate the change in Y direction

    #Calculate distance rho representing relative distance between the desired and the current position
    rho = np.sqrt((np.square(x_delta))+ (np.square(y_delta)))
    #Calculate angle gamma representing the angle between the global X-direction of the vehicle and rho
    gamma = np.arctan2(y_delta,x_delta)				
    
    if gamma < 0:
       gamma = gamma + (2*np.pi)

    #Calculate angle alpha between the local X-direction of the vehicle and rho 
    alpha = gamma - yawco
    #Calculate angle beta between rho and desired global X-direction of the vehicle
    beta = - alpha - yawco + (theta_des*(np.pi/180))
#########################################################################################################

#########################################################################################################
#Control function
def control():
   global rho		#Identify rho variable as global variable
   global beta		#Identify beta variable as global variable 
   global alpha		#Identify alpha variable as global variable
   global linear_v	#Identify linear_v variable as global variable
   global angular_v	#Identify angular_v variable as global variable 
   global Krho
   global Kalpha
   global Kbeta  
   print(rho)
   #Calculate controlled linear velocity and angular velocity
   if np.absolute(alpha) < np.pi/2: #Condition handles if desired position is infornt or behind the vehicle
       linear_v = Krho * rho
   else:
       linear_v = -Krho * rho

   angular_v = Kalpha * alpha + Kbeta * beta  
#########################################################################################################

#########################################################################################################
p=0
while 1 and not rospy.is_shutdown()  and p<len(y):
    #rospy.loginfo(x0)
    #Call the transformation function
    if rho<2 and p<(len(x)-1): 
	p=p+1

    x_des=x[p]
    y_des=y[p]
    theta_des=th[p]
    transformation(xcord, ycord, thetayaw)
    #Call the control function
    control()
    print(p)
    #Calculate the linear and angular velocities
    v = round(linear_v,2) 	#Linear Velocity
a    w = round(angular_v,2)	#Angular Velocity

    #Set the values of the Twist msg to be published
    vel_msg.linear.x = v  #Linear Velocity
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = w #Angular Velocity

    #ROS Code Publisher
    pub1.publish(vel_msg)	#Publish msg
    rate.sleep()		#Sleep with rate
#########################################################################################################

