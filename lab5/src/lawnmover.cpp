#include "lab5/lawnmover.hpp"
#include "ros/ros.h"

#include <cmath>

namespace turtle_control {

Lawnmover::Lawnmover() {
  sub_ = nh_.subscribe("turtle1/pose", 1, &Lawnmover::turtleCallback, this);
  pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  timer_ = nh_.createTimer(ros::Duration(0.01), &Lawnmover::ControlCycle, this);
}

Lawnmover::~Lawnmover() {}

void Lawnmover::turtleCallback(const turtlesim::Pose::ConstPtr &msg) {
  pos_x_ = msg->x;
  pos_y_ = msg->y;
  ang_ = msg->theta;
}

void Lawnmover::ControlCycle(const ros::TimerEvent &event) {
  switch (state_) {
    case UP:
      vel_.linear.x = SPEED_FORWARD_;
      vel_.angular.z = SPEED_STOP_;

      if (check_up_2_turnR()) {
        ROS_INFO("Change to TURNR");
        go_state(TURNR);
      }
      break;

    case DOWN:
      vel_.linear.x = SPEED_FORWARD_;
      vel_.angular.z = SPEED_STOP_;

      if (check_down_2_turnL()) {
        ROS_INFO("Change to TURNL");
        go_state(TURNL);
      }
      break;

    case TURNR:
      vel_.linear.x = SPEED_FORWARD2_;
      vel_.angular.z = -SPEED_TURN_;

      if (check_stop()) {
        ROS_INFO("Change to STOP");
        go_state(STOP);
      }
      if (check_turnR_2_down()) {
        ROS_INFO("Change to DOWN");
        go_state(DOWN);
      }
      break;

    case TURNL:
      vel_.linear.x = SPEED_FORWARD2_;
      vel_.angular.z = SPEED_TURN_;

      if (check_stop()) {
        ROS_INFO("Change to STOP");
        go_state(STOP);
      }
      if (check_turnL_2_up()) {
        ROS_INFO("Change to UP");
        go_state(UP);
      }
      break;

    case STOP:
      vel_.linear.x = SPEED_STOP_;
      vel_.angular.z = SPEED_STOP_;
  }
  pub_.publish(vel_);
}

void
Lawnmover::go_state(int new_state)
{
  state_ = new_state;
}

bool
Lawnmover::check_up_2_turnR()
{
  return pos_y_ > 10;
}

bool
Lawnmover::check_turnR_2_down()
{
  return fabs(ang_ - DIR_DOWN_) < 0.01;
}

bool
Lawnmover::check_down_2_turnL()
{
  return pos_y_ < 1;
}

bool
Lawnmover::check_turnL_2_up()
{
  return fabs(ang_ - DIR_UP_) < 0.01;
}

bool
Lawnmover::check_stop()
{
  return pos_x_ > 10;
}
} // namespace turtle_control
