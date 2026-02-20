#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

class FakeEncoder : public rclcpp::Node
{
public:
  FakeEncoder() : Node("fake_encoder")
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FakeEncoder::publishOdom, this));

    last_time_ = this->get_clock()->now();
  }

private:
  void publishOdom()
  {
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double linear_vel = 0.2;
    double angular_vel = 0.0;

    theta_ += angular_vel * dt;
    x_ += linear_vel * std::cos(theta_) * dt;
    y_ += linear_vel * std::sin(theta_) * dt;

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);

    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.angular.z = angular_vel;

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
    tf_msg.transform.rotation.z = std::sin(theta_ / 2.0);
    tf_msg.transform.rotation.w = std::cos(theta_ / 2.0);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeEncoder>());
  rclcpp::shutdown();
  return 0;
}