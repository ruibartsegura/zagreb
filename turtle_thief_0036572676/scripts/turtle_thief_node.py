#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Return distance between two points
def calc_distance(pos1, pos2):
    x = pos1.x - pos2.x
    y = pos1.y - pos2.y

    return math.sqrt(x * x + y * y)


class TurtleThief:

    def __init__(self):
        # Time at the init
        self.t_0 = rospy.Time.now().to_sec()

        # Initial position
        self.x_0 = 5.5
        self.y_0 = 5.5

        # Value of a as said in the question
        self.a = 5

        # Value of minimun distance to consider Mali is so close
        self.dist = 2

        # Position of the chest
        self.chest_position = Pose()
        self.chest_position.x = 5.5
        self.chest_position.y = 5.5

        # Publisher of position
        self.thief_target = rospy.Publisher('thief_target', Pose, queue_size=1)

        # Create subscriber to /turtle_pose
        rospy.Subscriber("pose", Pose, self.handle_alarm)

        rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_thief)

    def handle_alarm(self,data):
        distance = calc_distance(data,  self.chest_position)
        if (distance < self.dist):
            rospy.loginfo(f"Warning!! Intruder detected! Distance to the treasure chest: {distance}  TurtleSim units.")
    
    def publish_thief(self, _):
        # Get time
        t = rospy.Time.now().to_sec() - self.t_0

        # Get value of x and y with the formula of Gerono’s lemniscate
        self.pose = Pose()
        self.pose.x = self.a * math.sin(t) + self.x_0
        self.pose.y = self.a * math.sin(t) * math.cos(t) + self.y_0
        
        # Publishing the thief_target calculated before.
        self.thief_target.publish(self.pose)

def clamp(minvalue, value, maxvalue):
        return max(minvalue, min(value, maxvalue))



if __name__ == "__main__":
    rospy.init_node("turtle_thief_node")
    node = TurtleThief()
    rospy.spin()

