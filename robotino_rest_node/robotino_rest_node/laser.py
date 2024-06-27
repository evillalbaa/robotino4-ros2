#!/usr/bin/env python3
import requests
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanNode(Node):
    def __init__(self):
        super().__init__("robotino_laserscan")
        self.URL = "http://192.168.0.1/data/scan0"
        self.PARAMS = {'sid': 'robotino_rest_node'}
        self.publisher_ = self.create_publisher(LaserScan, "scan", 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # Publish at 5 Hz
        self.get_logger().info("LaserScan Node has been started")

    def timer_callback(self):
        scan_data = self.get_data()
        if scan_data:
            scan_msg = self.create_laserscan_message(scan_data)
            self.publisher_.publish(scan_msg)
            self.get_logger().info("Published LaserScan message")

    def get_data(self):
        try:
            response = requests.get(url=self.URL, params=self.PARAMS)
            if response.status_code == requests.codes.ok:
                return response.json()
            else:
                self.get_logger().warn("GET request to %s with params %s failed", self.URL, self.PARAMS)
        except requests.exceptions.RequestException as e:
            self.get_logger().error("%s", e)
        return None

    def create_laserscan_message(self, data):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = data["angle_min"]
        scan_msg.angle_max = data["angle_max"]
        scan_msg.angle_increment = data["angle_increment"]
        scan_msg.time_increment = data["time_increment"]
        scan_msg.scan_time = data["scan_time"]
        scan_msg.range_min = data["range_min"]
        scan_msg.range_max = data["range_max"]
        scan_msg.ranges = data["ranges"]
        scan_msg.intensities = data["intensities"]
        return scan_msg

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
