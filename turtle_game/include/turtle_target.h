#ifndef _TURTLE_TARGET_H_
#define _TURTLE_TARGET_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <turtle_target.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Spawn.h>

class TurleTargetClass {

private:
  std::string turtle_name;
  ros::NodeHandle nh_;
  ros::ServiceClient spawnClient;
  ros::ServiceClient client_pen;
  turtlesim::SetPen client_pen_srv;
  ros::Subscriber sub2_;
  double x_;
  double y_;

public:
  TurleTargetClass();
  void Initialize(std::string name, ros::NodeHandle nh);
  void PoseCallback(const turtlesim::Pose::ConstPtr &msg);
  double GetDistance(double x, double y);
  double GetAngle(double x, double y);
  void KillTurtle();
  void Spawn(std::string name);
};

#endif

