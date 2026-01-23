#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LiDARCropNode(Node):
    def __init__(self):
        super().__init__('lidar_crop_node')

        # Subscribe to original scan topic
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for cropped scan
        self.publisher = self.create_publisher(LaserScan, '/scan_cropped', 10)

        # Timer just to keep node alive even if no scan comes in yet
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("✅ LiDAR Crop Node started (Cropping 0° to +120°).")

    def scan_callback(self, msg):
        try:
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment

            self.get_logger().info(
                f"📡 Received scan: angle_min={angle_min:.2f}, angle_max={angle_max:.2f}, angle_increment={angle_increment:.4f}, total_points={len(msg.ranges)}"
            )

            # Crop from 0 to +120 degrees
            crop_min_angle = 0.0
            crop_max_angle = math.radians(180)  # ≈ 2.094 radians

            # Check that crop range is within available data
            if crop_min_angle < angle_min or crop_max_angle > angle_max:
                self.get_logger().warn("⚠️ Crop range is outside available scan angles. Skipping this frame.")
                return

            # Calculate crop indices
            start_index = int((crop_min_angle - angle_min) / angle_increment)
            end_index = int((crop_max_angle - angle_min) / angle_increment)

            self.get_logger().info(f"✂️ Cropping scan from index {start_index} to {end_index}.")

            # Create cropped scan message
            cropped_scan = LaserScan()
            cropped_scan.header = msg.header
            cropped_scan.angle_min = crop_min_angle
            cropped_scan.angle_max = crop_max_angle
            cropped_scan.angle_increment = msg.angle_increment
            cropped_scan.time_increment = msg.time_increment
            cropped_scan.scan_time = msg.scan_time
            cropped_scan.range_min = msg.range_min
            cropped_scan.range_max = msg.range_max
            cropped_scan.ranges = msg.ranges[start_index:end_index]
            cropped_scan.intensities = msg.intensities[start_index:end_index] if msg.intensities else []

            self.publisher.publish(cropped_scan)
            self.get_logger().info("✅ Published cropped scan.")
        except Exception as e:
            self.get_logger().error(f"❌ Error in scan_callback: {str(e)}")

    def timer_callback(self):
        self.get_logger().debug("🌀 Waiting for scan data...")

def main(args=None):
    rclpy.init(args=args)
    node = LiDARCropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
