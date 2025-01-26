#include <turtle_target.h>

TurleTargetClass::TurleTargetClass() { srand(time(NULL)); }

void TurleTargetClass::Initialize(std::string name, ros::NodeHandle nh) {
  nh_ = nh;

  sub2_ = nh_.subscribe("/" + name + "/pose", 1,
                        &TurleTargetClass::PoseCallback, this);

  Spawn(name);
}

void TurleTargetClass::PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
  x_ = msg->x;
  y_ = msg->y;
}

void TurleTargetClass::KillTurtle() {
  ros::service::waitForService("/kill");
  ros::ServiceClient killClient = nh_.serviceClient<turtlesim::Kill>("kill");
  turtlesim::Kill kill_srv;
  kill_srv.request.name = "turtle2";
  killClient.call(kill_srv);
  Spawn("turtle2");
}

double TurleTargetClass::GetDistance(double x, double y) {
  return sqrt(pow(x - x_, 2) + pow(y - y_, 2));
}

double TurleTargetClass::GetAngle(double x, double y) {
  return atan2(y_ - y, x_ - x);
}

void TurleTargetClass::Spawn(std::string name) {
  ros::service::waitForService("/spawn");
  spawnClient = nh_.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn spawn_srv;
  spawn_srv.request.x = (rand() % 100) * (9) / 100 + 0.5;
  spawn_srv.request.y = (rand() % 100) * (9) / 100 + 0.5;
  spawn_srv.request.theta = (rand() % 315) / 100.0;
  spawn_srv.request.name = name;
  spawnClient.call(spawn_srv);

  ros::service::waitForService("/" + name + "/set_pen");
  client_pen = nh_.serviceClient<turtlesim::SetPen>("/" + name + "/set_pen");
  client_pen_srv.request.r = 0;
  client_pen_srv.request.g = 0;
  client_pen_srv.request.b = 0;
  client_pen_srv.request.width = 0;
  client_pen_srv.request.off = 1;
  client_pen.call(client_pen_srv);
}

