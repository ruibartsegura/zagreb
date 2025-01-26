#include "node_programming/turtle_lawnmover.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_lawnmower_node");

    turtle_control::TurtleLawnmover ttMower;

    ros::spin();

    return 0;
}
