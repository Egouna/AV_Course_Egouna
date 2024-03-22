#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
import tf_transformations


class TurtleNavigationNode(Node):

    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our navigation has started")

        self.initial_pose_publisher=self.create_publisher(
            PoseStamped,"/initialpose",10)
        
        self.pose_publisher=self.create_publisher(
            PoseWithCovarianceStamped,"/goalpose",10)
        
        self.odom_pose_listener=self.create_subscription(
            Odometry,"/odom",self.robot_controller,10)
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0

        qq = tf_transformations.quaternion_from_euler(0,0,0)
        initial_pose.pose.pose.orientation.x = qq[0]
        initial_pose.pose.pose.orientation.y = qq[1]
        initial_pose.pose.pose.orientation.z = qq[2]
        initial_pose.pose.pose.orientation.w = qq[3]
        self.initial_pose_publisher.publish(initial_pose)

    def robot_controller(self,scan:LaserScan):
        pass


def main(args=None):
    rclpy.init(args=args)
    node=TurtleNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()