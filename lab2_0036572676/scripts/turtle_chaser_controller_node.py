#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class TurtleChaserControllerNode:

    def __init__(self):
        # Initialize the last received chaser and target poses to None, since we do not know them at the beginning.
        self.turtle_pose = Pose()
        self.mouse_pose = Pose()

        # Forward & Angular gain set as private parameters with 1 and 2 as default value
        self.forward_gain = rospy.get_param('~forward_gain', 1)
        self.angular_gain = rospy.get_param('~angular_gain', 2)

        # Print the gain factor ROS parameters to confirm they have been set correctly.
        rospy.loginfo('Forward Gain = %d Angular Gain =%d', self.forward_gain, self.angular_gain)

        # Publisher of velocity of tge chaser
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.angular.z = 0

        # Create a subscriber for pose msg, one for the chaser other for the target
        rospy.Subscriber("mouse_pose", Pose, self.target_callback)
        rospy.Subscriber("turtle_pose", Pose, self.chaser_callback)

        # Little sleep to ensure that the rosbag records the simulation properly
        rospy.sleep(1)

    def chaser_callback(self,data):
        self.turtle_pose = data

    def target_callback(self,data):
        self.mouse_pose = data
        self.chase()


    def chase(self):
        dist, ang = self.get_dif_position()

        if dist > 0.3:
            self.vel.linear.x = dist * self.forward_gain
            self.vel.angular.z = ang * self.angular_gain
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0

        self.vel_pub.publish(self.vel)

    def get_dif_position(self):
        x = self.mouse_pose.x - self.turtle_pose.x
        y = self.mouse_pose.y - self.turtle_pose.y

        dist = math.sqrt(x * x + y * y)
        ang = math.atan2(y, x) - self.turtle_pose.theta
        ang = wrap_to_pi(ang)

        return dist, ang

if __name__ == "__main__":
    rospy.init_node("turtle_chaser_controller")
    node = TurtleChaserControllerNode()
    rospy.spin()

