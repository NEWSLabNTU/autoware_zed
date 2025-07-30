#include <rclcpp/rclcpp.hpp>
#include "autoware_zed/autoware_zed_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware_zed::AutowareZedNode>());
  rclcpp::shutdown();
  return 0;
}