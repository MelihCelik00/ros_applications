#!/usr/bin/env python

# Author: Melih Safa Celik
# Keyboard control code for turtlebot3's burger model


import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios # tty and termios are imported for linux users

# defining restrictions for linear and angular velocities for model 'burger'
max_linear_v = 0.30
max_angular_v = 2.50

linear_vel_step_size = 0.02
angular_vel_step_size = 0.1
# message for user
msg = """
	Control your robot with:

	- w: forward
	- a: left
	- s: backward
	- d: right
	- h: force stop

	- Velocity limit for turtlebot3 burger -> linear: 0.30, angular: 2.50

 	- Press CTRL + C for quit!
"""

fail_msg = "Can't communicate!!"

class ControlTB3():	
	def __init__(self):
		self.command_sent = 0
    	self.target_linear_vel   = 0.0
    	self.target_angular_vel  = 0.0
    	self.control_linear_vel  = 0.0
    	self.control_angular_vel = 0.0

		while not rospy.is_shutdown():
			print(msg)
			self.curr_key = getKey() # Check pulled keyboard inputs

			if (self.curr_key == 'w'):
				self.target_linear_vel = checkLinVelocityLimits(self.target_linear_vel + self.linear_vel_step_size)
                self.command_sent += 1
                print(velocities(self.target_linear_vel,self.target_angular_vel))
			
			elif (self.curr_key == 'a'):
				self.target_angular_vel = checkAngVelocityLimits(self.target_angular_vel + self.angular_vel_step_size)
                self.command_sent += 1
                print(velocities(self.target_linear_vel,self.target_angular_vel))
			
			elif (self.curr_key == 's'):
				self.target_linear_vel = checkLinVelocityLimits(self.target_linear_vel - self.linear_vel_step_size)
				self.command_sent += 1
				print(velocities(self.target_linear_vel, self.target_angular_vel))

			elif (self.curr_key == 'd'):
				self.target_angular_vel = checkAngVelocityLimits(self.target_angular_vel - self.angular_vel_step_size)
				self.command_sent += 1
				print(velocities(self.target_linear_vel, self.target_angular_vel))

			elif (self.curr_key == 'h'):
				self.target_linear_vel   = 0.0
                self.control_linear_vel  = 0.0
                self.target_angular_vel  = 0.0
                self.control_angular_vel = 0.0
				print(velocities(self.target_linear_vel, self.target_angular_vel))

			else:
				if (self.curr_key == '\x03'):
					break

			if (command_sent == 20):
				print(msg)
				command_sent = 0

			twist = Twist()

			control_linear_vel = controlTarget(control_linear_vel, target_linear_vel, (linear_vel_step_size/2.0))
			twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0;

			control_angular_vel = controlTarget(control_angular_vel, target_angular_vel, (angular_vel_step_size/2.0))
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

			pub.publish(twist)

	def getKey():
		tty.setraw(sys.stdin.fileno())
    	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    	if rlist:
        	key = sys.stdin.read(1)
    	else:
        	key = ''

    	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    	return key

	def velocities(target_linear_vel, target_angular_vel):
		print("Current linear velocity: {},\nCurrent angular velocity: {}".format(target_linear_vel, target_angular_vel))

	def restrictions(input, low, high):
	    if input < low:
	      input = low
	    elif input > high:
	      input = high
	    else:
	      input = input

	    return input

	def checkLinVelocityLimits(vel): # Other models can be added
	    if (turtlebot3_model == "burger"):
	      vel = restrictions(vel, -max_linear_v, max_linear_v)

	    return vel


	def checkAngVelocityLimits(vel):
		if (turtlebot3_model == "burger"):
			vel = restrictions(vel, -max_angular_v, max_linear_v)

		return vel

	def controlTarget(output, input, slop):
	    if input > output:
	        output = min( input, output + slop )
	    elif input < output:
	        output = max( input, output - slop )
	    else:
	        output = input


if __name__ == '__main__':
    try:
        ControlTB3()
    except:
    	print(fail_msg)
