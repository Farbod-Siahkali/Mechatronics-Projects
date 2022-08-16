#!/usr/bin/env python3

### ROS libraries
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

### Other libraries
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from math import sin, asin
import math

class RobotInterface():
	def __init__(self, lin_speed=0.15, ang_speed=np.pi/10):
		self.lin_speed = lin_speed
		self.ang_speed = ang_speed

		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.odometry_callback)

		self.curr_ang_pose = 0

	def odometry_callback(self, msg):
		self.twist_linear_x = msg.twist.twist.linear.x
		self.twist_linear_y = msg.twist.twist.linear.y
		self.twist_linear_z = msg.twist.twist.linear.z
		self.twist_angular_x = msg.twist.twist.angular.x
		self.twist_angular_y = msg.twist.twist.angular.y
		self.twist_angular_z = msg.twist.twist.angular.z
		
		self.position_x = msg.pose.pose.position.x
		self.position_y = msg.pose.pose.position.y
		self.position_z = msg.pose.pose.position.z
		self.orien = msg.pose.pose.orientation
		
		self.set_euler_angles()

	def set_euler_angles(self):
		t0 = +2.0 * (self.orien.w * self.orien.x + self.orien.y * self.orien.z)
		t1 = +1.0 - 2.0 * (self.orien.x * self.orien.x + self.orien.y * self.orien.y)
		self.roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (self.orien.w * self.orien.y - self.orien.z * self.orien.x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		self.pitch_y = math.asin(t2)

		t3 = +2.0 * (self.orien.w * self.orien.z + self.orien.x * self.orien.y)
		t4 = +1.0 - 2.0 * (self.orien.y * self.orien.y + self.orien.z * self.orien.z)
		self.yaw_z = math.atan2(t3, t4)

		# print("roll_x =", self.roll_x, "\tpitch_y =", self.pitch_y, "\tyaw_z =", self.yaw_z)
			
	def rotate(self, degrees):
		vel_msg = Twist()

		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = np.sign(degrees-self.yaw_z)*self.ang_speed

		t0 = rospy.Time.now().to_sec()
		while abs(degrees-self.yaw_z)>+np.pi/20:
			self.cmd_vel_pub.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()

		vel_msg.angular.z = 0
		self.cmd_vel_pub.publish(vel_msg)
		self.curr_ang_pose = self.curr_ang_pose + degrees
		
		print("angular movement done!")

	def move(self, distance):
		vel_msg = Twist()

		vel_msg.linear.x = np.sign(distance)*self.lin_speed
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0

		curr_distance = 0
		t0 = rospy.Time.now().to_sec()
		while curr_distance<abs(distance):
			self.cmd_vel_pub.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			curr_distance = (t1-t0)*self.lin_speed

		vel_msg.linear.x = 0
		self.cmd_vel_pub.publish(vel_msg)

		print("linear movement done!")


rospy.init_node('robot_controller', anonymous=True)
robot = RobotInterface()

rospy.sleep(5)

goal = [1.7, 1]

x = np.linspace(-2, 2, 80)
y = np.linspace(-2, 2, 80)
goal_potential = np.zeros((80,80))
blocks_potential = np.zeros((80,80))
potential = np.zeros((80,80))

alpha = 5
beta = 100
s = 0.4

obstacles = [[-1.1, -1.1], [-1.1, 0], [-1.1, 1.1] , [0, -1.1], [0,0] , [0, 1.1], [1.1, -1.1], [1.1, 0], [1.1, 1.1]]

r = 0.15
l = 0.5

for i, xx in enumerate(x):
    for j, yy in enumerate(y):
        for ob in obstacles:
            goal_potential[i][j] = 0.5*alpha*math.sqrt(pow(xx-goal[0],2) + pow(yy-goal[1],2))
            if (math.sqrt(pow(xx-ob[0],2) + pow(yy-ob[1],2)))<r:
                blocks_potential[i][j] = 25
            
            elif (math.sqrt(pow(xx-ob[0],2) + pow(yy-ob[1],2)))>s:
                blocks_potential[i][j] += 0
            
            else:
                blocks_potential[i][j] +=  0.5*beta*(s+r-math.sqrt(pow(xx-ob[0],2) + pow(yy-ob[1],2)))

potential = blocks_potential+goal_potential

plt.figure()
plt.pcolor(potential)
plt.colorbar()
plt.show()

pos_now = (6, 40)
orientation = 0

from itertools import product

def neighbours(cell):
    for c in product(*(range(n-1, n+2) for n in cell)):
        if c != cell and all(0 <= n < 80 for n in c):
            yield c

path = []
path.append(pos_now)
path.append(pos_now)
path.append(pos_now)

while (pos_now != (74,60)):
 
    nears = list(neighbours((pos_now[0],pos_now[1])))
    flag = 0

    for near in nears:
        if flag == 0:
            temp = potential[near[0]][near[1]]
            temp_index = near
            flag = 1
        
        else:
            if potential[near[0]][near[1]] <= temp:
                if near == path[-2] or near == path[-3]:
                    continue
                temp = potential[near[0]][near[1]]
                temp_index = near

    path.append(temp_index)
    pos_now = temp_index

path.pop(0)
path.pop(0)

for p in path:
    potential[p[0]][p[1]] = 40

plt.figure()
plt.pcolor(potential)
plt.colorbar()
plt.show()

print("founded a route")
print("Starting to move...")

for i, p in enumerate(path):
    diff_y = path[i+1][0]-p[0]
    diff_x = path[i+1][1]-p[1]

    if diff_y == 1 and diff_x == 0: #(0,1) up
        if robot.curr_ang_pose == 0:
            robot.move(0.05)
        else:
            robot.rotate(-robot.curr_ang_pose)
            robot.move(0.05)

    elif diff_y == 1 and diff_x == 1: #(1,1) up right
        if robot.curr_ang_pose == -np.pi/4:
            robot.move(0.07071)
        else:
            robot.rotate(-np.pi/4-robot.curr_ang_pose)
            robot.move(0.07071) 

    elif diff_y == 1 and diff_x == -1: #(-1,1) up left
        if robot.curr_ang_pose == np.pi/4:
            robot.move(0.07071)
        else:
            robot.rotate(np.pi/4-robot.curr_ang_pose)
            robot.move(0.07071) 

    elif diff_y == 0 and diff_x == 1: #(1,0) right
        if robot.curr_ang_pose == -np.pi/2:
            robot.move(0.05)
        else:
            robot.rotate(-np.pi/2-robot.curr_ang_pose)
            robot.move(0.05) 

    elif diff_y == 0 and diff_x == -1: #(-1,0) left
        if robot.curr_ang_pose == np.pi/2:
            robot.move(0.05)
        else:
            robot.rotate(np.pi/2-robot.curr_ang_pose)
            robot.move(0.05) 

print("Reached at the destination.")