#include "autoware_zed_converter/autoware_zed_converter_node.hpp"
#include <cmath>
#include <algorithm>

namespace autoware_zed_converter
{

AutowareZedConverterNode::AutowareZedConverterNode(const rclcpp::NodeOptions & options)
: Node("autoware_zed_converter_node", options)
{
  // Declare parameters
  this->declare_parameter("input_topic", "/zed/zed_node/obj_det/objects");
  this->declare_parameter("output_topic", "/perception/object_recognition/objects");
  this->declare_parameter("use_tracking_velocity", true);
  this->declare_parameter("existence_probability_threshold", 0.5);

  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  use_tracking_velocity_ = this->get_parameter("use_tracking_velocity").as_bool();
  existence_probability_threshold_ = this->get_parameter("existence_probability_threshold").as_double();

  // Initialize label mapping from ZED to Autoware
  // ZED labels -> Autoware ObjectClassification labels
  label_map_ = {
    {"Person", autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN},
    {"person", autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN},
    {"PERSON", autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN},
    {"Vehicle", autoware_perception_msgs::msg::ObjectClassification::CAR},
    {"vehicle", autoware_perception_msgs::msg::ObjectClassification::CAR},
    {"VEHICLE", autoware_perception_msgs::msg::ObjectClassification::CAR},
    {"Car", autoware_perception_msgs::msg::ObjectClassification::CAR},
    {"car", autoware_perception_msgs::msg::ObjectClassification::CAR},
    {"Truck", autoware_perception_msgs::msg::ObjectClassification::TRUCK},
    {"truck", autoware_perception_msgs::msg::ObjectClassification::TRUCK},
    {"Bus", autoware_perception_msgs::msg::ObjectClassification::BUS},
    {"bus", autoware_perception_msgs::msg::ObjectClassification::BUS},
    {"Bicycle", autoware_perception_msgs::msg::ObjectClassification::BICYCLE},
    {"bicycle", autoware_perception_msgs::msg::ObjectClassification::BICYCLE},
    {"Motorcycle", autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE},
    {"motorcycle", autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE}
  };

  // Create subscriber
  zed_objects_sub_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
    input_topic_, 10,
    std::bind(&AutowareZedConverterNode::objectsCallback, this, std::placeholders::_1));

  // Create publisher
  autoware_objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    output_topic_, 10);

  RCLCPP_INFO(this->get_logger(), 
    "ZED to Autoware Converter initialized:\n"
    "  Input topic: %s\n"
    "  Output topic: %s",
    input_topic_.c_str(), output_topic_.c_str());
}

void AutowareZedConverterNode::objectsCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
{
  // Create output message
  autoware_perception_msgs::msg::DetectedObjects autoware_msg;
  autoware_msg.header = msg->header;
  // Always use the input frame_id
  autoware_msg.header.frame_id = msg->header.frame_id;

  // Transform each object
  for (const auto & zed_obj : msg->objects) {
    // Convert confidence to existence probability (0.0 - 1.0)
    float existence_prob = zed_obj.confidence / 100.0f;
    
    // Skip objects below threshold
    if (existence_prob < existence_probability_threshold_) {
      continue;
    }

    // Convert object
    auto autoware_obj = convertZedObjectToAutoware(zed_obj);
    autoware_obj.existence_probability = existence_prob;
    
    // Add to output
    autoware_msg.objects.push_back(autoware_obj);
  }

  // Publish
  autoware_objects_pub_->publish(autoware_msg);
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Converted %zu ZED objects to %zu Autoware objects",
    msg->objects.size(), autoware_msg.objects.size());
}

autoware_perception_msgs::msg::ObjectClassification::_label_type 
AutowareZedConverterNode::mapZedLabelToAutoware(const std::string & zed_label) const
{
  auto it = label_map_.find(zed_label);
  if (it != label_map_.end()) {
    return it->second;
  }
  
  // Default to UNKNOWN if no mapping found
  RCLCPP_WARN_ONCE(this->get_logger(), 
    "Unknown ZED label: %s, mapping to UNKNOWN", zed_label.c_str());
  return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
}

autoware_perception_msgs::msg::DetectedObject 
AutowareZedConverterNode::convertZedObjectToAutoware(const zed_msgs::msg::Object & zed_obj) const
{
  autoware_perception_msgs::msg::DetectedObject autoware_obj;

  // Classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = mapZedLabelToAutoware(zed_obj.label);
  classification.probability = zed_obj.confidence / 100.0f;
  autoware_obj.classification.push_back(classification);

  // Kinematics - Position (validate for NaN/Inf)
  autoware_obj.kinematics.pose_with_covariance.pose.position.x = 
    isValidNumber(zed_obj.position[0]) ? zed_obj.position[0] : 0.0;
  autoware_obj.kinematics.pose_with_covariance.pose.position.y = 
    isValidNumber(zed_obj.position[1]) ? zed_obj.position[1] : 0.0;
  autoware_obj.kinematics.pose_with_covariance.pose.position.z = 
    isValidNumber(zed_obj.position[2]) ? zed_obj.position[2] : 0.0;
  
  // Orientation - ZED doesn't provide orientation, so we use identity quaternion
  autoware_obj.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  autoware_obj.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  autoware_obj.kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  autoware_obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  autoware_obj.kinematics.orientation_availability = 
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;

  // Position covariance (ZED provides 6 values: xx, yy, zz, xy, xz, yz)
  if (zed_obj.position_covariance.size() >= 6) {
    // Initialize covariance to zero
    std::fill(autoware_obj.kinematics.pose_with_covariance.covariance.begin(),
              autoware_obj.kinematics.pose_with_covariance.covariance.end(), 0.0);
    
    // Convert to 6x6 covariance matrix (only position part) with NaN/Inf validation
    autoware_obj.kinematics.pose_with_covariance.covariance[0] = 
      isValidNumber(zed_obj.position_covariance[0]) ? zed_obj.position_covariance[0] : 0.0;  // xx
    autoware_obj.kinematics.pose_with_covariance.covariance[7] = 
      isValidNumber(zed_obj.position_covariance[1]) ? zed_obj.position_covariance[1] : 0.0;  // yy
    autoware_obj.kinematics.pose_with_covariance.covariance[14] = 
      isValidNumber(zed_obj.position_covariance[2]) ? zed_obj.position_covariance[2] : 0.0; // zz
    autoware_obj.kinematics.pose_with_covariance.covariance[1] = 
      isValidNumber(zed_obj.position_covariance[3]) ? zed_obj.position_covariance[3] : 0.0;  // xy
    autoware_obj.kinematics.pose_with_covariance.covariance[6] = 
      isValidNumber(zed_obj.position_covariance[3]) ? zed_obj.position_covariance[3] : 0.0;  // yx
    autoware_obj.kinematics.pose_with_covariance.covariance[2] = 
      isValidNumber(zed_obj.position_covariance[4]) ? zed_obj.position_covariance[4] : 0.0;  // xz
    autoware_obj.kinematics.pose_with_covariance.covariance[12] = 
      isValidNumber(zed_obj.position_covariance[4]) ? zed_obj.position_covariance[4] : 0.0; // zx
    autoware_obj.kinematics.pose_with_covariance.covariance[8] = 
      isValidNumber(zed_obj.position_covariance[5]) ? zed_obj.position_covariance[5] : 0.0;  // yz
    autoware_obj.kinematics.pose_with_covariance.covariance[13] = 
      isValidNumber(zed_obj.position_covariance[5]) ? zed_obj.position_covariance[5] : 0.0; // zy
    autoware_obj.kinematics.has_position_covariance = true;
  } else {
    autoware_obj.kinematics.has_position_covariance = false;
  }

  // Velocity (if tracking is available and enabled) with NaN/Inf validation
  if (use_tracking_velocity_ && zed_obj.tracking_available && 
      zed_obj.tracking_state == 1) { // 1 = OK
    autoware_obj.kinematics.twist_with_covariance.twist.linear.x = 
      isValidNumber(zed_obj.velocity[0]) ? zed_obj.velocity[0] : 0.0;
    autoware_obj.kinematics.twist_with_covariance.twist.linear.y = 
      isValidNumber(zed_obj.velocity[1]) ? zed_obj.velocity[1] : 0.0;
    autoware_obj.kinematics.twist_with_covariance.twist.linear.z = 
      isValidNumber(zed_obj.velocity[2]) ? zed_obj.velocity[2] : 0.0;
    autoware_obj.kinematics.has_twist = true;
    autoware_obj.kinematics.has_twist_covariance = false; // ZED doesn't provide velocity covariance
  } else {
    autoware_obj.kinematics.has_twist = false;
    autoware_obj.kinematics.has_twist_covariance = false;
  }

  // Shape - Use bounding box with NaN/Inf validation
  autoware_obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  autoware_obj.shape.dimensions.x = 
    isValidNumber(zed_obj.dimensions_3d[0]) ? zed_obj.dimensions_3d[0] : 0.1; // width (default to small value)
  autoware_obj.shape.dimensions.y = 
    isValidNumber(zed_obj.dimensions_3d[1]) ? zed_obj.dimensions_3d[1] : 0.1; // height
  autoware_obj.shape.dimensions.z = 
    isValidNumber(zed_obj.dimensions_3d[2]) ? zed_obj.dimensions_3d[2] : 0.1; // length

  return autoware_obj;
}

bool AutowareZedConverterNode::isValidNumber(float value) const
{
  return std::isfinite(value);
}

} // namespace autoware_zed_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_zed_converter::AutowareZedConverterNode)