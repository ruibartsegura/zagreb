#include <turtle_target.h>

class MainTurtleClass {

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  TurleTargetClass turtle_target;

  double threshold;
  double Kp_v;
  double Kp_theta;
  double Ki_v;
  double Ki_theta;
  double integrator_v;
  double integrator_a;
  double sum_v;
  double sum_a;

public:
  MainTurtleClass(ros::NodeHandle nh) {

    // Set up the node handle
    nh_ = nh;
    turtle_target = TurleTargetClass();
    turtle_target.Initialize("turtle2", nh);
    RemovePen();
    InitializeValues();

    // Set up subscribers and publishers
    sub_ =
        nh_.subscribe("/turtle1/pose", 1, &MainTurtleClass::PoseCallback, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  }

  void InitializeValues() {
    integrator_v = 0.0;
    integrator_a = 0.0;
    sum_v = 0.0;
    sum_a = 0.0;

    // Read parameters form.yaml file
    nh_.param<double>("/game/Kp_v", Kp_v, 0.0);
    nh_.param<double>("/game/Kp_theta", Kp_theta, 0.0);
    nh_.param<double>("/game/Ki_v", Ki_v, 0.0);
    nh_.param<double>("/game/Ki_theta", Ki_theta, 0.0);
    nh_.param<double>("/game/threshold", threshold, 0.0);
  }

  void RemovePen() {
    // Remove pen from turtle1
    ros::service::waitForService("/turtle1/set_pen");
    ros::ServiceClient client_pen =
        nh_.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen client_pen_srv;
    client_pen_srv.request.r = 0;
    client_pen_srv.request.g = 0;
    client_pen_srv.request.b = 0;
    client_pen_srv.request.width = 0;
    client_pen_srv.request.off = 1;
    client_pen.call(client_pen_srv);
  }

  void PoseCallback(const turtlesim::Pose::ConstPtr &msg) {

    // Get euclidean and angle distance between turtle 1 and 2
    double d_euclidean = turtle_target.GetDistance(msg->x, msg->y);
    double d_angle = turtle_target.GetAngle(msg->x, msg->y) - msg->theta;

    if (d_angle > M_PI) {
      d_angle -= 2 * M_PI;
    } else if (d_angle < -M_PI) {
      d_angle += 2 * M_PI;
    }

    // TODO
    // Fill following expressions
    sum_v = sum_v + d_euclidean;
    sum_a = sum_a + d_angle;
    integrator_v = d_euclidean  * Kp_v + Ki_v * sum_v;
    integrator_a = d_angle * Kp_theta + Ki_theta * sum_a;

    if (d_euclidean < threshold) {

      ROS_INFO("Turtle1 got the target!");
      turtle_target.KillTurtle();

      integrator_a = 0.0;
      integrator_v = 0.0;

      geometry_msgs::Twist msg;
      msg.linear.x = 0;
      msg.angular.z = 0;
      pub_.publish(msg);
      ros::Duration(0.5).sleep();
    } else {
      VelocityController(d_euclidean, d_angle);
    }
  }

  void VelocityController(double dist_pos, double dist_theta) {
    geometry_msgs::Twist msg;

    // TODO
    // Fill following expressions
    msg.linear.x = integrator_v;
    msg.linear.y = 0;
    msg.angular.z = integrator_a;
    pub_.publish(msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_game");
  ros::NodeHandle nh;
  MainTurtleClass main_turtle(nh);
  ros::spin();
  return (0);
}

