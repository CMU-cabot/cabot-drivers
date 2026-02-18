#include "cabot_shared_control/shared_control_node.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cabot_shared_control::SharedControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
