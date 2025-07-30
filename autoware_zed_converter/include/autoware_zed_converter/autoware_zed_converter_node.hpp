#ifndef AUTOWARE_ZED_CONVERTER_NODE_HPP
#define AUTOWARE_ZED_CONVERTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <string>
#include <memory>
#include <unordered_map>

namespace autoware_zed_converter
{

class AutowareZedConverterNode : public rclcpp::Node
{
public:
  explicit AutowareZedConverterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void objectsCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);
  
  autoware_perception_msgs::msg::ObjectClassification::_label_type 
    mapZedLabelToAutoware(const std::string & zed_label) const;
  
  autoware_perception_msgs::msg::DetectedObject 
    convertZedObjectToAutoware(const zed_msgs::msg::Object & zed_obj) const;
    
  bool isValidNumber(float value) const;

  // Subscribers
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr zed_objects_sub_;
  
  // Publishers
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr autoware_objects_pub_;
  
  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  bool use_tracking_velocity_;
  double existence_probability_threshold_;
  
  // Label mapping from ZED to Autoware
  std::unordered_map<std::string, uint8_t> label_map_;
};

} // namespace autoware_zed_converter

#endif // AUTOWARE_ZED_CONVERTER_NODE_HPP