#!/usr/bin/env python3
import rclpy
import requests
from rclpy.node import Node
from std_msgs.msg import Bool

class BumperNode(Node):
    def __init__(self):
        super().__init__("robotino_bumper")
        timer_period = 1 # seconds
        self.URL = "http://192.168.0.1/data/bumper"
        self.PARAMS = {'sid': 'robotino_rest_node'}

        self.bumper_pub_ = self.create_publisher(Bool, "bumper", 1)
        self.timer_ = self.create_timer(timer_period, self.get_bumper_data) 
        self.get_logger().info("Bumper Node has been started")        

    def get_bumper_data(self):
        msg = Bool()
        try:
            r = requests.get(url=self.URL, params=self.PARAMS,timeout=5)
            if r.status_code == requests.codes.ok:
                data = r.json()
                msg.data = data["value"]
                self.get_logger().info(str(data))
                self.bumper_pub_.publish(msg)
            else:
                self.get_logger().warn("get from %s with params %s failed", self.URL, self.PARAMS)
        except requests.exceptions.RequestException as e:
            self.get_logger().error("%s", e) 

def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()
    rclpy.spin(node)
    rclpy.shutdown()
