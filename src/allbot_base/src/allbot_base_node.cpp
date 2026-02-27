#include "allbot_base/allbot_base.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AllbotBase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
