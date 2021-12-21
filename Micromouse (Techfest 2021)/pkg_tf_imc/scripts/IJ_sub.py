#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Int32MultiArray as Arr32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

I_data = 0
J_data = 0
dir_data = 0
pi = 3.1415926535897
yaw = 0
x_dist = 0
y_dist = 0
X_cen = []
Y_cen = []
wall = 0.012
cell = 0.168

def clbk_IJ(msg):
    global dir_data
    global I_data
    global J_data
    dir_data = msg.data[0]
    I_data = msg.data[2]
    J_data = msg.data[1]

def clbk_odom(msg):
    global x_dist
    global y_dist
    global yaw
    position_ = msg.pose.pose.position
    x_dist = position_.x
    y_dist = position_.y
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

def pri(theta):
    if -2*pi < theta and theta <= -1*pi :
	return (theta + 2*pi)
    elif -1*pi < theta and theta <= pi :
	return (theta)
    elif pi < theta and theta <= 2*pi :
	return (theta - 2*pi)
    else :
	print("gadbad ho gayi")
	return (theta)	

def centers():
    X1,X2 = [],[]
    global cell
    global wall
    X1.append(round(cell/2 + wall/2,5))
    X2.append(round(-1*cell/2 - wall/2,5))
    for i in range(7):
        X1.append(round(X1[-1] + cell + wall,5))
        X2.append(round(X2[-1] - cell - wall,5))
    #X_cen,Y_cen = [],[]
    global X_cen
    global Y_cen
    X2.reverse()
    X_cen = X2 + X1
    Y_cen = X_cen[:]
    Y_cen.reverse()
    print(X_cen)
    print(Y_cen)
    return X_cen, Y_cen

def IJ(x,y):
    I,J = -1,-1
    global cell
    for i in range(16):
        if (X_cen[i] - cell/2 < x and x < X_cen[i] + cell/2):
            I=i
            break
    for j in range(16):
        if (Y_cen[j] - cell/2 < y and y < Y_cen[j] + cell/2):
            J=j
            break
    return I,J

def rotate(angle, angular_speed, pub):
    rate = rospy.Rate(50) # 40hz
    yaw0 = yaw
    msg1 = Twist()
    msg2 = Twist()
    msg2.linear.x = 0.0
    msg2.angular.z = 0.0
    speed_x = 0.0
    msg1.linear.x = speed_x
    if angle > 0 :
	msg1.angular.z = angular_speed
    	while pri(yaw-yaw0) < angle :
	    #print("in rotate loop")
	    #print(yaw, yaw0, yaw-yaw0, pri(yaw-yaw0), angle)
	    pub.publish(msg1)
	    rate.sleep()
    elif angle < 0 :
	msg1.angular.z = -1*angular_speed
	while pri(yaw-yaw0) > angle :
            #print("in rotate loop")
	    #print(yaw, yaw0, yaw-yaw0, pri(yaw-yaw0), angle)
	    pub.publish(msg1)
	    rate.sleep()
    for i in range(10):
	pub.publish(msg2)
	#print("in rotate stop loop")
        rate.sleep()

def dist(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def move_to(i, j, pub):
    x_final,y_final = X_cen[i], Y_cen[j]
    print("going to :")
    print(j,i)
    print(x_final, y_final)
    rate = rospy.Rate(50) # 40hz
    angle = pri(pi - math.atan2(x_final-x_dist, y_final-y_dist))
    rel_angle = pri(angle - yaw)
    rotate(rel_angle, 20*pi/360, pub)
    msg2 = Twist()
    msg2.linear.x = 0.0
    msg2.angular.z = 0.0
    threshold = 0.01
    while dist(x_final, y_final, x_dist, y_dist) > threshold :
	#print("in move loop")
	msg1 = Twist()
	msg1.linear.x = 0.08
        ang = pri(pri(pi - math.atan2(x_final-x_dist, y_final-y_dist)) - yaw)
	msg1.angular.z = 0.08*ang
	pub.publish(msg1)
	rate.sleep()
    for i in range(10) :
	pub.publish(msg2)
	#print("in move stop loop")
        rate.sleep()
	 
def main():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_IJ = rospy.Subscriber('/moveToCell', Arr32, clbk_IJ) 
    rospy.init_node('cmd_move', anonymous=True)
    rate = rospy.Rate(50) # 40hz

    centers()
    I_curr, J_curr = 0,0

    while not rospy.is_shutdown():
        i,j = I_data, J_data
	if not ( i<16 and i>=0 and j<16 and j>=0):
	    print(i,j)
	    print("i,j values out of range")
	if I_curr != i or J_curr != j :
	    I_curr, J_curr = i,j
	    move_to(i,j,pub)
	if i == -1 :
	    break 

    while not rospy.is_shutdown():
	msg1 = Twist()
        speed_z = 0.0000000000000000
	speed_x = 0.0000000000000000
        msg1.linear.x = speed_x
        msg1.angular.z = speed_z
        pub.publish(msg1)
        rate.sleep()
    
if __name__ == '__main__':
    main()
