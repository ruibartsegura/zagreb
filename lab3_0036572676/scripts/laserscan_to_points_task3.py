#!/usr/bin/env python3
import rospy
import math

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LaserScanToPoints:

    def __init__(self):
        self.last_timestamp = rospy.Time(0)

        # Publisher for the marker
        self.pub = rospy.Publisher('points', Marker, queue_size=1)

        rospy.Subscriber("scan", LaserScan, self.laser_callback)
        
    def laser_callback(self, scan):
        # Create the marker
        marker = Marker()
        marker.header = scan.header
        marker.type = Marker.POINTS
        marker.pose.orientation.w = 1.0
        marker.color.r = 0.2
        marker.color.b = 0.8
        marker.color.g = 0.2
        marker.color.a = 0.5
        # For Marker.POINTS, scale.x is width, scale.y is height
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.points = []

        # Clear the points if the recording restarts
        if scan.header.stamp < self.last_timestamp:
                # Clear the trajectory if the timestamp has jumped backwards
                marker.points.clear()
                rospy.loginfo("Timestamp has jumped backwards. Clearing the trajectory marker.")


        # Values of the scan to get the coords
        for range_obs in scan.ranges:
            idx_obs = scan.ranges.index(range_obs)
            angle_obs = scan.angle_min + idx_obs * scan.angle_increment

            # Coords x and of the detected obstacle
            p = Point()
            p.x = range_obs * math.cos(angle_obs)
            p.y = range_obs * math.sin(angle_obs)
            
            marker.points.append(p)

        self.pub.publish(marker)
        self.last_timestamp = scan.header.stamp


if __name__ == "__main__":
    rospy.init_node("laser_scan_to_points")
    node = LaserScanToPoints()
    rospy.spin()

