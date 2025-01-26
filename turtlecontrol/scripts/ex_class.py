#!/usr/bin/env python3
#Must import rospy and msgs
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Node example class.
class NodeExample():
    # Must have callback for msgs
    def pose_callback(self,data):
        self.pose_x = data.x
        return self.pose_x


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
        self.cmd_vel.linear.x = 1
        self.cmd_vel.angular.z = 0
        self.pose_x = 0
        # Create a subscriber for color msg
        rospy.Subscriber("pose", Pose, self.pose_callback)


    def run(self):
        # Main while loop.
        while not rospy.is_shutdown():
            if self.pose_x > 6:
                self.cmd_vel.linear.x = -1
            elif self.pose_x < 5:
                self.cmd_vel.linear.x = 1

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

