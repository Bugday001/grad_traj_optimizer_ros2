#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class MinimalPublisher(Node):
    '''
    发布器节点类
    '''
    def __init__(self):
        super().__init__('path_pub')
        self.publisher_ = self.create_publisher(Path, 'target_path', 10)
        msg = Path()
        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        msg.poses.append(pose)
        pose = PoseStamped()
        pose.pose.position.x = 10.0
        pose.pose.position.y = 30.0
        pose.pose.position.z = 2.0
        msg.poses.append(pose)
        self.publisher_.publish(msg)
        

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)

    # 创建节点
    minimal_publisher = MinimalPublisher()

    # 运行节点
    rclpy.spin(minimal_publisher)

    # 销毁节点，退出ROS2
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
