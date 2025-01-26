#!/usr/bin/env python3
import rospy
import math
import tf2_ros

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped

class LaserScanToPoints:
    def __init__(self):
        # Private param: FIXED FRAME
        self.fixed_frame = rospy.get_param('~fixed_frame_id', 'map')
        self.accumulate_points = rospy.get_param('~accumulate_points', False)
        self.accumulate_every_n = rospy.get_param('~accumulate_every_n', 50)

        print(f"Starting the LaserScan to points node.")
        print(f"fixed_frame_id: {self.fixed_frame}")
        print(f"accumulate_points: {self.accumulate_points}")
        print(f"accumulate_every_n: {self.accumulate_every_n}")

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Counter for scans 
        self.counter = 0

        # Previuos scan
        self.previous_scan = None

        # Previous timestamp
        self.last_timestamp = rospy.Time(0)

        # Create the marker
        self.marker = Marker()
        self.marker.header.frame_id = self.fixed_frame
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.b = 0.8
        self.marker.color.g = 0.6
        self.marker.color.a = 0.5
        # For Marker.POINTS, scale.x is width, scale.y is height
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.points = []

        # Publisher for the marker
        self.pub = rospy.Publisher('points', Marker, queue_size=1)

        # Subscriber for LaserScan messages
        rospy.Subscriber("scan", LaserScan, self.laser_callback)
        
    def laser_callback(self, scan):
        self.counter += 1
        # Handle timestamp jumps, I change the function because it doesn't work in my code
        if scan.header.stamp < self.last_timestamp:
            self.marker.header.stamp = scan.header.stamp
            self.last_timestamp = scan.header.stamp
            self.marker.points.clear()
            self.tf_buffer.clear()
            print('Timestamp has jumped backwards. Clearing the buffer.')
            return

        if self.accumulate_points == False:
            self.marker.points.clear()
        else:
            # If the counter is different of the accumulate skips the scan
            if self.counter % self.accumulate_every_n != 0:
                return

        # Values of the scan to get the coords
        for range_obs in scan.ranges:
            # Skip invalid points (range == 0 or out of range)
            if range_obs == 0 or range_obs > scan.range_max:
                continue

            idx_obs = scan.ranges.index(range_obs)
            angle_obs = scan.angle_min + idx_obs * scan.angle_increment

            # Coords x and of the detected obstacle
            p = Point()
            p.x = range_obs * math.cos(angle_obs)
            p.y = range_obs * math.sin(angle_obs)
            try:
                # Lookup the latest transform from the laser frame to the fixed frame
                laser_2_fixed_frame = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    scan.header.frame_id,
                    scan.header.stamp
                      # Timeout duration
                )

                translation = laser_2_fixed_frame.transform.translation
                quaternion = laser_2_fixed_frame.transform.rotation
                # Recover the yaw angle 
                yaw = 2 * math.atan2(quaternion.z, quaternion.w)

                # Create the transformed point
                transformed_point = Point()
                transformed_point.x = p.x * math.cos(yaw) - p.y * math.sin(yaw) + translation.x
                transformed_point.y = p.x * math.sin(yaw) + p.y * math.cos(yaw) + translation.y
                self.marker.points.append(transformed_point)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                rospy.logwarn(ex)
                return # To exit early from the callback

        self.pub.publish(self.marker)
        self.last_timestamp = scan.header.stamp


if __name__ == "__main__":
    rospy.init_node("laser_scan_to_points")
    node = LaserScanToPoints()
    rospy.spin()
