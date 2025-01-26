#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def commander(v,w):
    while not rospy.is_shutdown():
        vel=Twist()
        vel.linear.x = v
        vel.angular.z = w
        pub.publish(vel)
        rospy.sleep(1.0)

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.init_node('publisher')
    v=-1; w = 2
    try:
        while not rospy.is_shutdown():
            commander(v,w)
    except rospy.ROSInterruptException:
        pass
