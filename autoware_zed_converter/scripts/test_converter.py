#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
from autoware_perception_msgs.msg import DetectedObjects
import time


class ConverterTester(Node):
    def __init__(self):
        super().__init__('converter_tester')
        
        # Statistics
        self.zed_msg_count = 0
        self.autoware_msg_count = 0
        self.last_zed_msg_time = None
        self.last_autoware_msg_time = None
        
        # Subscribers
        self.zed_sub = self.create_subscription(
            ObjectsStamped,
            '/zed_multi/zed_front/obj_det/objects',
            self.zed_callback,
            10
        )
        
        self.autoware_sub = self.create_subscription(
            DetectedObjects,
            '/perception/object_recognition/objects/front_test',
            self.autoware_callback,
            10
        )
        
        # Timer for periodic status
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Converter Tester started')
        self.get_logger().info('Listening to:')
        self.get_logger().info('  ZED input: /zed_multi/zed_front/obj_det/objects')
        self.get_logger().info('  Autoware output: /perception/object_recognition/objects/front_test')
    
    def zed_callback(self, msg):
        self.zed_msg_count += 1
        self.last_zed_msg_time = self.get_clock().now()
        
        # Log details of first few messages
        if self.zed_msg_count <= 3:
            self.get_logger().info(f'ZED Message #{self.zed_msg_count}:')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            self.get_logger().info(f'  Number of objects: {len(msg.objects)}')
            
            for i, obj in enumerate(msg.objects):
                self.get_logger().info(f'  Object {i}:')
                self.get_logger().info(f'    Label: {obj.label} (ID: {obj.label_id})')
                self.get_logger().info(f'    Confidence: {obj.confidence}%')
                self.get_logger().info(f'    Position: [{obj.position[0]:.2f}, {obj.position[1]:.2f}, {obj.position[2]:.2f}]')
                self.get_logger().info(f'    Dimensions: [{obj.dimensions_3d[0]:.2f}, {obj.dimensions_3d[1]:.2f}, {obj.dimensions_3d[2]:.2f}]')
                if obj.tracking_available:
                    self.get_logger().info(f'    Velocity: [{obj.velocity[0]:.2f}, {obj.velocity[1]:.2f}, {obj.velocity[2]:.2f}]')
    
    def autoware_callback(self, msg):
        self.autoware_msg_count += 1
        self.last_autoware_msg_time = self.get_clock().now()
        
        # Log details of first few messages
        if self.autoware_msg_count <= 3:
            self.get_logger().info(f'Autoware Message #{self.autoware_msg_count}:')
            self.get_logger().info(f'  Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
            self.get_logger().info(f'  Number of objects: {len(msg.objects)}')
            
            for i, obj in enumerate(msg.objects):
                self.get_logger().info(f'  Object {i}:')
                self.get_logger().info(f'    Existence probability: {obj.existence_probability:.2f}')
                if obj.classification:
                    label_names = {
                        0: 'UNKNOWN', 1: 'CAR', 2: 'TRUCK', 3: 'BUS',
                        4: 'TRAILER', 5: 'MOTORCYCLE', 6: 'BICYCLE', 7: 'PEDESTRIAN'
                    }
                    label = obj.classification[0].label
                    label_name = label_names.get(label, f'UNKNOWN({label})')
                    self.get_logger().info(f'    Classification: {label_name} (prob: {obj.classification[0].probability:.2f})')
                
                pos = obj.kinematics.pose_with_covariance.pose.position
                self.get_logger().info(f'    Position: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]')
                
                dim = obj.shape.dimensions
                self.get_logger().info(f'    Dimensions: [{dim.x:.2f}, {dim.y:.2f}, {dim.z:.2f}]')
                
                if obj.kinematics.has_twist:
                    vel = obj.kinematics.twist_with_covariance.twist.linear
                    self.get_logger().info(f'    Velocity: [{vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}]')
    
    def print_status(self):
        self.get_logger().info('='*50)
        self.get_logger().info(f'Status Report:')
        self.get_logger().info(f'  ZED messages received: {self.zed_msg_count}')
        self.get_logger().info(f'  Autoware messages received: {self.autoware_msg_count}')
        
        current_time = self.get_clock().now()
        if self.last_zed_msg_time:
            zed_age = (current_time - self.last_zed_msg_time).nanoseconds / 1e9
            self.get_logger().info(f'  Last ZED message: {zed_age:.1f}s ago')
        else:
            self.get_logger().info('  No ZED messages received yet')
            
        if self.last_autoware_msg_time:
            autoware_age = (current_time - self.last_autoware_msg_time).nanoseconds / 1e9
            self.get_logger().info(f'  Last Autoware message: {autoware_age:.1f}s ago')
        else:
            self.get_logger().info('  No Autoware messages received yet')
        
        if self.zed_msg_count > 0:
            conversion_rate = (self.autoware_msg_count / self.zed_msg_count) * 100
            self.get_logger().info(f'  Conversion rate: {conversion_rate:.1f}%')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    tester = ConverterTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()