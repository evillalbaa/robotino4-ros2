#!/usr/bin/env python3
import requests
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OmnidriveNode(Node):
    def __init__(self):
        super().__init__("robotino_omnidrive")
        self.URL = "http://192.168.0.1/data/omnidrive"
        self.PARAMS = {'sid': 'robotino_rest_node'}
        self.subscriber_ = self.create_subscription(
            Twist, "cmd_vel", self.lisener_callback, 10
        )
        self.get_logger().info("Omnidrive Node has been started")

    def lisener_callback(self, msg: Twist):
        self.get_logger().info('Received Twist: %f %f %f' % (msg.linear.x, msg.linear.y, msg.angular.z))
        pdata = {'vx': msg.linear.x, 'vy': msg.linear.y, 'omega': msg.angular.z}
        self.post_data(pdata)
        
    def post_data(self, pdata):
        try:
            r = requests.post(url=self.URL, params=self.PARAMS, data=json.dumps(pdata))
            if r.status_code != requests.codes.ok:
                self.get_logger().warn("post to %s with params %s failed", self.URL, self.PARAMS)
        except requests.exceptions.RequestException as e:
            self.get_logger().error("%s", e)

def main(args=None):
    rclpy.init(args=args)
    node = OmnidriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
