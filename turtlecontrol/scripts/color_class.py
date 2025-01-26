#!/usr/bin/env python3
#Must import rospy and msgs
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Color

# Node example class.
class NodeExample():
	# Must have callback for msgs
	def color_callback(self,data):
		rospy.loginfo(rospy.get_caller_id()+' Color I see is r = %d g =%d b =%d', data.r,data.g,data.b)


	# Must have __init__(self) function for a class
	# similar to a C++ class constructor.
	def __init__(self):
		# Create a publisher for turtle commands
		self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		# Define publisher rate		
		self.rate = rospy.Rate(10) # 10hz
		# Set the message to publish as command.
		# self.variable means you can access it from class fnc
		self.cmd_vel = Twist()
		# Initialize message variables.
		self.cmd_vel.linear.x = -1.1
		self.cmd_vel.angular.z = -1.1
		# Create a subscriber for color msg
		rospy.Subscriber("color", Color, self.color_callback)


	def run(self):
		# Main while loop.
		while not rospy.is_shutdown():
			# Publish our command!
			self.pub.publish(self.cmd_vel)
			self.rate.sleep()

if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('pyclass')
	# Go to class functions that do all the heavy lifting.
	try:
		ne = NodeExample()
		ne.run()
	except rospy.ROSInterruptException: pass

