#ifndef AUTOWARE_ZED_NODE_HPP
#define AUTOWARE_ZED_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <memory>
#include <unordered_map>

namespace autoware_zed
{

class AutowareZedNode : public rclcpp::Node
{
public:
  explicit AutowareZedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  
  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  bool use_tracking_velocity_;
  double existence_probability_threshold_;
  
  // Label mapping from ZED to Autoware
  std::unordered_map<std::string, uint8_t> label_map_;
};

} // namespace autoware_zed

#endif // AUTOWARE_ZED_NODE_HPP