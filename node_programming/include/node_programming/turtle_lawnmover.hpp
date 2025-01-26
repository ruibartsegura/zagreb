#ifndef TURTLE_LAWNMOVER_HPP
#define TURTLE_LAWNMOVER_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

namespace turtle_control {

class TurtleLawnmover {
public:
    TurtleLawnmover();
    ~TurtleLawnmover();

    void turtleCallback(const turtlesim::Pose::ConstPtr &msg);

private:
    void ControlCycle(const ros::TimerEvent &event);

    ros::NodeHandle nh_;

    // Subscriber
    ros::Subscriber sub_;

    // Publisher
    ros::Publisher pub_;
    geometry_msgs::Twist vel_;

    // Timer controlcycle
    ros::Timer timer_;
};

} // namespace turtle_control

#endif // TURTLE_LAWNMOVER_HPP
