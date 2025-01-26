#!/usr/bin/env python3

import rospy
import tf2_ros
import math

from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from tf2_geometry_msgs import do_transform_pose

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TrajectoryVisualization:

    def __init__(self):
        now = rospy.Time.now()

        # If there is a previus remap these lines will delete it and then use the news ones
        self.fixed_frame = rospy.get_param('~fixed_frame_id', 'map')
        self.robot_frame = rospy.get_param('~robot_frame_id', 'base_link')
        
        print(f"Starting the trajectory visualization node.")
        print(f"fixed_frame_id: {self.fixed_frame}")
        print(f"robot_frame_id: {self.robot_frame}")

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Last known position & time
        self.last_position = None
        self.last_timestamp = rospy.Time(0)

        # Marker setup
        self.marker = Marker()
        self.marker.header.frame_id = self.fixed_frame  # Use the fixed frame
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.8
        self.marker.color.b = 0.2
        self.marker.color.a = 1.0

        self.marker.scale.x = 0.1  # Width of points
        self.marker.scale.y = 0.1  # Height of points
        self.marker.points = []

        # Publisher for Marker
        self.marker_pub = rospy.Publisher("robot_positions", Marker, queue_size=10)


        # Timer Callback
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.robot_pos_callback, reset=True)

    def robot_pos_callback(self, _):
        try:
            # Lookup the latest transform from the fixed frame to the robot frame
            transform = self.tf_buffer.lookup_transform(self.fixed_frame, self.robot_frame, rospy.Time(0))
            position = transform.transform

            if transform.header.stamp < self.last_timestamp:
                # Clear the trajectory if the timestamp has jumped backwards
                self.marker.points.clear()
                rospy.loginfo("Timestamp has jumped backwards. Clearing the trajectory marker.")

            # Check if the robot position has changed since the last update
            if (self.last_position == None or 
                    position.translation.x != self.last_position.translation.x or
                    position.translation.y != self.last_position.translation.y):
                # Create a new Point message for the marker
                new_point = Point()
                new_point.x = position.translation.x
                new_point.y = position.translation.y

                # Append the new point to the marker points
                self.marker.points.append(new_point)
                rospy.loginfo(f"Added new point to trajectory: {new_point}")

                # Update the marker header with the latest transform's header
                self.marker.header.frame_id = transform.header.frame_id
                self.marker.header.stamp = transform.header.stamp

                # Publish the updated marker
                self.marker_pub.publish(self.marker)

            self.last_position = position
            self.last_timestamp = transform.header.stamp

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn(ex)
            return # To exit early from the callback

if __name__ == "__main__":
    rospy.init_node("trayectory_visualization")
    node = TrajectoryVisualization()
    rospy.spin()
