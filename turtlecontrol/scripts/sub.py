#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(data):
    #rospy.loginfo(data)
    rospy.loginfo(rospy.get_caller_id()+" I saw turtlebot" + " at x = %d y =%d", 
                                                                data.x,data.y)

def subscriber():
    rospy.init_node('subscriber',anonymous=True)
    rospy.Subscriber("pose", Pose, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass