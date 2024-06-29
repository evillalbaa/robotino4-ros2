#!/usr/bin/env python3
import requests
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf_transformations import quaternion_from_euler

class OdometryNode(Node):
    def __init__(self):
        super().__init__("robotino_odometry")
        self.URL = "http://192.168.0.1/data/odometry"
        self.PARAMS = {'sid': 'robotino_rest_node'}
        self.publisher_ = self.create_publisher(Odometry, "odom", 10)
        self.timer = self.create_timer(0.005, self.timer_callback)  # Publish at 200 Hz
        self.get_logger().info("Odometry Node has been started")

    def timer_callback(self):
        odom_data = self.get_data()
        if odom_data:
            odom_msg = self.create_odometry_message(odom_data)
            self.publisher_.publish(odom_msg)
            self.get_logger().info("Published Odometry message")

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

    def create_odometry_message(self, data):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # Pose
        pose = Pose()
        pose.position = Point(data[0], data[1], 0.0)
        quat = quaternion_from_euler(0.0, 0.0, data[2])
        #pose.orientation = Quaternion(*quat)
        #odom_msg.pose.pose = pose
        
        # Twist
        twist = Twist()
        twist.linear.x = data[3]
        twist.linear.y = data[4]
        twist.angular.z = data[5]
        odom_msg.twist.twist = twist
        
        return odom_msg

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
