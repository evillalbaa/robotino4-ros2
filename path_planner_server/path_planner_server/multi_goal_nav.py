#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from nav_msgs.msg import Path
import math

class MultiGoalNav(Node):
    def __init__(self):
        super().__init__('multi_goal_nav')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = [
            PoseStamped(
                header=Header(frame_id='map'),
                pose=Pose(
                    position=Point(x=1.0, y=1.0, z=0.0),
                    orientation=Quaternion(w=1.0)
                )
            ),
            PoseStamped(
                header=Header(frame_id='map'),
                pose=Pose(
                    position=Point(x=2.0, y=2.0, z=0.0),
                    orientation=Quaternion(w=1.0)
                )
            ),
            PoseStamped(
                header=Header(frame_id='map'),
                pose=Pose(
                    position=Point(x=3.0, y=1.0, z=0.0),
                    orientation=Quaternion(w=1.0)
                )
            )
        ]
        self.current_goal_idx = 0
        self.path_publisher = self.create_publisher(
            Path,
            'robot_trajectory',
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'

    def send_next_goal(self):
        if self.current_goal_idx >= len(self.goals):
            self.get_logger().info('All goals reached!')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.goals[self.current_goal_idx]

        self.get_logger().info(f'Sending goal {self.current_goal_idx + 1}: {goal.pose.pose.position}')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.current_goal_idx += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')
        # Update trajectory
        self.trajectory.poses.append(feedback.current_pose)
        self.path_publisher.publish(self.trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNav()
    node.send_next_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
