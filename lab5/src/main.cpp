#include "lab5/lawnmover.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "lawnmower_node");

    turtle_control::Lawnmover ttMower;

    ros::spin();

    return 0;
}
