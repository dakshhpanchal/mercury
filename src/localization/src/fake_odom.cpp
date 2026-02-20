#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <chrono>

class FakeOdom : public rclcpp::Node
{
public:
  FakeOdom() : Node("fake_odom")
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FakeOdom::update, this));

    last_time_ = this->get_clock()->now();

    setupKeyboard();

    RCLCPP_INFO(this->get_logger(), "Fake Odom Started (WASD Control)");
    RCLCPP_INFO(this->get_logger(), "W=Forward  S=Back  A=Left  D=Right");
  }

  ~FakeOdom()
  {
    restoreKeyboard();
  }

private:
  void setupKeyboard()
  {
    tcgetattr(STDIN_FILENO, &original_term_);
    termios raw = original_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  }

  void restoreKeyboard()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_term_);
  }

  char getKey()
  {
    char c;
    if (read(STDIN_FILENO, &c, 1) > 0)
      return c;
    return 0;
  }

  void update()
  {
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0 || dt > 0.1)
      dt = 0.05;

    last_time_ = current_time;

    char key = getKey();

    double linear_vel = 0.0;
    double angular_vel = 0.0;

    const double LIN_SPEED = 0.3;
    const double ANG_SPEED = 1.0;

    switch(key)
    {
      case 'w': linear_vel = LIN_SPEED; break;
      case 's': linear_vel = -LIN_SPEED; break;
      case 'a': angular_vel = ANG_SPEED; break;
      case 'd': angular_vel = -ANG_SPEED; break;
      default: break;
    }

    theta_ += angular_vel * dt;
    x_ += linear_vel * std::cos(theta_) * dt;
    y_ += linear_vel * std::sin(theta_) * dt;

    publishOdom(current_time, linear_vel, angular_vel);
  }

  void publishOdom(rclcpp::Time current_time,
                   double linear_vel,
                   double angular_vel)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(theta_/2.0);
    odom.pose.pose.orientation.w = std::cos(theta_/2.0);

    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.angular.z = angular_vel;

    for(int i=0;i<36;i++)
    {
      odom.pose.covariance[i] = 0.0;
      odom.twist.covariance[i] = 0.0;
    }

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[35] = 0.02;

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[35] = 0.02;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = std::sin(theta_/2.0);
    tf_msg.transform.rotation.w = std::cos(theta_/2.0);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;
  termios original_term_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}