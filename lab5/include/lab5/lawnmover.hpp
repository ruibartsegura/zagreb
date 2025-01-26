#ifndef LAWNMOVER_HPP
#define LAWNMOVER_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

namespace turtle_control {

class Lawnmover {
public:
  Lawnmover();
  ~Lawnmover();

  void turtleCallback(const turtlesim::Pose::ConstPtr &msg);

private:
  void ControlCycle(const ros::TimerEvent &event);

  ros::NodeHandle nh_;

  void go_state(int new_state);

  bool check_up_2_turnR();
  bool check_down_2_turnL();
  bool check_turnR_2_down();
  bool check_turnL_2_up();
  bool check_stop();


  static const int UP = 0;
  static const int DOWN = 1;
  static const int TURNR = 2;
  static const int TURNL = 3;
  static const int STOP = 4;

  int state_;

  double SPEED_FORWARD_ = 3;
  double SPEED_FORWARD2_ = 0.3;
  double SPEED_TURN_ = 0.8;
  double SPEED_STOP_ = 0;
  
  float DIR_UP_ = M_PI/2;
  float DIR_DOWN_ = -M_PI/2;

  // Subscriber
  ros::Subscriber sub_;
  float pos_x_;
  float pos_y_;
  float ang_;

  // Publisher
  ros::Publisher pub_;
  geometry_msgs::Twist vel_;

  // Timer controlcycle
  ros::Timer timer_;
};

} // namespace turtle_control

#endif // LAWNMOVER_HPP
