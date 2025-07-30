#include <rclcpp/rclcpp.hpp>
#include "autoware_zed_converter/autoware_zed_converter_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware_zed_converter::AutowareZedConverterNode>());
  rclcpp::shutdown();
  return 0;
}