import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped


class Publisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.subscriber_ = self.create_subscription(PointStamped, '/clicked_point', self.callback, 1)
        self.subscriber_  # prevent unused variable warning
    
    def callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.point.x, msg.point.y, msg.point.z))
        self.publish(msg.point.x, msg.point.y)

    def publish(self,x,y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = '/map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        self.get_logger().info('Publishing  Initial Position  \n X= %f \n Y= %f '% (msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
