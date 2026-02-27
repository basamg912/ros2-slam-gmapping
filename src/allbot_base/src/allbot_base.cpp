#include <allbot_base/allbot_base.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

AllbotBase::AllbotBase()
    : Node("allbot_base_node"),
      linear_velocity_x_(0), linear_velocity_y_(0), angular_velocity_z_(0),
      last_vel_time_(0), vel_dt_(0), x_pos_(0), y_pos_(0), heading_(0) {
  
  // Initialize parameters
  this->declare_parameter("linear_scale", 1.0);
  // Removed serial parameters

  linear_scale_ = this->get_parameter("linear_scale").as_double();
  
  // Publishers
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("raw_odom", 50);
  odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscriber
  vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "raw_vel", 10, std::bind(&AllbotBase::velocityCallback, this, std::placeholders::_1));

  last_vel_time_ = this->now();
}

void AllbotBase::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  rclcpp::Time current_time = this->now();

  linear_velocity_x_ = msg->linear.x * linear_scale_;
  linear_velocity_y_ = msg->linear.y * linear_scale_;
  angular_velocity_z_ = msg->angular.z;

  vel_dt_ = (current_time - last_vel_time_).seconds();
  last_vel_time_ = current_time;

  double delta_heading = angular_velocity_z_ * vel_dt_; // radians
  double delta_x = (linear_velocity_x_ * cos(heading_) -
                    linear_velocity_y_ * sin(heading_)) *
                   vel_dt_; // m
  double delta_y = (linear_velocity_x_ * sin(heading_) +
                    linear_velocity_y_ * cos(heading_)) *
                   vel_dt_; // m

  // calculate current position of the robot
  x_pos_ += delta_x;
  y_pos_ += delta_y;
  heading_ += delta_heading;

  // calculate robot's heading in quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, heading_);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  // Publish TF
  // odom_broadcaster_->sendTransform(odom_trans); // Uncomment if TF needed

  // Publish Odometry
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // Position
  odom.pose.pose.position.x = x_pos_;
  odom.pose.pose.position.y = y_pos_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  
  // Velocity
  odom.twist.twist.linear.x = linear_velocity_x_;
  odom.twist.twist.linear.y = linear_velocity_y_;
  odom.twist.twist.angular.z = angular_velocity_z_;

  // Covariance
  // Pose covariance (x, y, z, roll, pitch, yaw)
  odom.pose.covariance[0] = 0.001;  // x
  odom.pose.covariance[7] = 0.001;  // y
  odom.pose.covariance[35] = 0.001; // yaw

  // Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
  odom.twist.covariance[0] = 0.001;  // vx
  odom.twist.covariance[7] = 0.001;  // vy
  odom.twist.covariance[35] = 0.001; // vyaw

  odom_publisher_->publish(odom);
}
