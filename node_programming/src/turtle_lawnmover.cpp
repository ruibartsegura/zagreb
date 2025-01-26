#include "node_programming/turtle_lawnmover.hpp"

namespace turtle_control {

TurtleLawnmover::TurtleLawnmover() {
    sub_ = nh_.subscribe("turtle1/pose", 1, &TurtleLawnmover::turtleCallback, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    timer_ = nh_.createTimer(ros::Duration(0.1), &TurtleLawnmover::ControlCycle, this);
}

TurtleLawnmover::~TurtleLawnmover() {}

void TurtleLawnmover::turtleCallback(const turtlesim::Pose::ConstPtr &msg) {
    ROS_INFO("Turtlemover[%f, %f, %f]", msg->x, msg->y, msg->theta);
}

void TurtleLawnmover::ControlCycle(const ros::TimerEvent &event) {
    vel_.linear.x = 1;
    vel_.angular.z = 1;
    pub_.publish(vel_);
}

} // namespace turtle_control
