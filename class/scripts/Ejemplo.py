#!/usr/bin/env python3
import rospy
import math
import tf2_ros

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped

class LaserScanToPoints:
    def __init__(self):
        # Private param FIXED FRAME
        self.fixed_frame = rospy.get_param('~fixed_frame_id', 'map')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Create the marker
        self.marker = Marker()
        self.marker.header.frame_id = self.fixed_frame
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.b = 0.8
        self.marker.color.g = 0.2
        self.marker.color.a = 0.5
        self.marker.scale.x = 0.3  # Point size
        self.marker.scale.y = 0.3  # Point size
        self.marker.points = []

        # Publisher of the marker
        self.pub = rospy.Publisher('points', Marker, queue_size=1)

        # Subscriber for LaserScan messages
        rospy.Subscriber("scan", LaserScan, self.laser_callback)
        
    def laser_callback(self, scan):
        # Handle timestamp jumps
        if scan.header.stamp < self.marker.header.stamp:
            rospy.logwarn('Timestamp has jumped backwards. Clearing the buffer.')
            self.marker.header.stamp = scan.header.stamp
            self.marker.points.clear()
            self.tf_buffer.clear()
            return

        # Clear the previous points for each new scan
        self.marker.points.clear()

        # Process the laser scan data
        for idx_obs, range_obs in enumerate(scan.ranges):
            # Skip invalid points (range == 0 or out of range)
            if range_obs == 0 or range_obs > scan.range_max:
                continue

            # Calculate the angle for this laser scan point
            angle_obs = scan.angle_min + idx_obs * scan.angle_increment

            # Convert polar to Cartesian coordinates in the laser frame
            p = Point()
            p.x = range_obs * math.cos(angle_obs)
            p.y = range_obs * math.sin(angle_obs)

            try:
                # Lookup the transform from laser frame to fixed frame (e.g., map)
                transform = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    scan.header.frame_id,
                    scan.header.stamp,
                    rospy.Duration(0.2)  # Timeout duration
                )

                # Extract translation and quaternion (rotation) from the transform
                translation = transform.transform.translation
                quaternion = transform.transform.rotation

                # Recover the yaw angle (rotation around the Z axis) from the quaternion
                yaw = 2 * math.atan2(quaternion.z, quaternion.w)

                # Apply the transform to the laser point
                transformed_x = p.x * math.cos(yaw) - p.y * math.sin(yaw) + translation.x
                transformed_y = p.x * math.sin(yaw) + p.y * math.cos(yaw) + translation.y

                # Create the transformed point
                transformed_point = Point()
                transformed_point.x = transformed_x
                transformed_point.y = transformed_y
                transformed_point.z = 0  # Assume 2D space (no Z transformation)

                # Add the transformed point to the marker
                self.marker.points.append(transformed_point)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                rospy.logwarn("TF exception: %s", ex)
                return  # Return early if a transform lookup fails

        # Publish the transformed points
        self.pub.publish(self.marker)

if __name__ == "__main__":
    rospy.init_node("laser_scan_to_points")
    node = LaserScanToPoints()
    rospy.spin()
    