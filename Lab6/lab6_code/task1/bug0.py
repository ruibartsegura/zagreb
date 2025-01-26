import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class BugAlgorithm:
    def __init__(self, goal_x, goal_y):
        rospy.init_node('bug_algorithm')
        
        # Goal position
        self.goal_x = goal_x
        self.goal_y = goal_y

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.obstacle_detected = False

        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def scan_callback(self, msg):
        distance_threshold = 3.0  # Consecutive points below this distance are considered part of an obstacle
        obstacle_threshold = 1.0  # If the average distance of a cluster is below this, mark as an obstacle

        ranges = msg.ranges
        cluster = []  
        clusters = []  

        
        for r in ranges:
            if r < distance_threshold:
                cluster.append(r)
            else:
                if len(cluster) > 0:
                    clusters.append(cluster)
                    cluster = [] 

        if len(cluster) > 0:
            clusters.append(cluster)

        
        obstacle_detected = False
        for cluster in clusters:
            average_distance = sum(cluster) / len(cluster)
            print(f"Cluster average distance: {average_distance}")

            
            if average_distance < obstacle_threshold:
                obstacle_detected = True
                rospy.loginfo("Obstacle detected!")
                break

        # Update the obstacle detection flag
        self.obstacle_detected = obstacle_detected
        if not obstacle_detected:
            rospy.loginfo("No obstacle detected.")



    def odom_callback(self, msg):
        # Get the robot's current position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def move_to_goal(self):
        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown():
            # Calculate the distance to the goal
            distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

            #TODO
            #Fill the following
            if obstacle_detected:
                continue
            
            if distance_to_goal == 0:
            



            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    bug = BugAlgorithm(goal_x=5.0, goal_y=5.0) 
    bug.move_to_goal()

