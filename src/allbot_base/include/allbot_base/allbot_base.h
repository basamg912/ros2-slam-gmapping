#ifndef ALLBOT_BASE_H
#define ALLBOT_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <allbot_interface/msg/velocities.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>

class AllbotBase : public rclcpp::Node {
public:
  AllbotBase();
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  // rclcpp::TimerBase::SharedPtr timer_; // Removed as readSerial is gone

  // Serial related members removed
  
  float linear_scale_;
  float linear_velocity_x_;
  float linear_velocity_y_;
  float angular_velocity_z_;
  rclcpp::Time last_vel_time_;
  float vel_dt_;
  float x_pos_;
  float y_pos_;
  float heading_;
};

#endif
