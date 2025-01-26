#!/usr/bin/env python3
import rospy
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def correct_point(p):
    angle = math.atan2(p.y, p.x) * 180 / math.pi
    r = math.sqrt(p.x**2 + p.y**2)

    point = Point()
    point.x = r * math.cos(angle)
    point.y = r * math.sin(angle)
    return point

class FixPoints:

    def __init__(self):
        # Publisher
        self.pub = rospy.Publisher('points_corrected', Marker, queue_size=1)

        rospy.Subscriber("points", Marker, self.points_callback)

    def points_callback(self, data):
        corrected_points = []
        for point in data.points:
            corrected_points.append(correct_point(point))
        data.points = corrected_points
        data.header.frame_id = "corrected_points"
        self.pub.publish(data)



if __name__ == "__main__":
    rospy.init_node("fix_points")
    node = FixPoints()
    rospy.spin()

