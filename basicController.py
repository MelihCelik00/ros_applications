#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	q_rotation = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([q_rotation.x, q_rotation.y, q_rotation.z, q_rotation.w])

rospy.init_node("speed_controller")

subscriber = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

twist = Twist()

r = rospy.Rate(10)

xgoal = input("Enter goal for x: ")
ygoal = input("Enter goal for y: ")


goal = Point()
goal.x = xgoal #x goal
goal.y = ygoal #y goal

while not rospy.is_shutdown():

	dist_x = goal.x - x
	dist_y = goal.y - y

	measured_angle = atan2(dist_y, dist_x)

	if (abs(measured_angle - theta) > 0.1):

		twist.linear.x = 0.0
		twist.angular.z = 0.5
		print("linear x: ",twist.linear.x," angular z: ",twist.angular.z)
	else:
		twist.linear.x = 2.0
		twist.angular.z = 0.0
		print("linear x: ",twist.linear.x," angular z: ",twist.angular.z)

	publisher.publish(twist)
	r.sleep()

