#!/usr/bin/python3

from LAB1_package.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node


class EKF_odom_node(Node):
    def __init__(self):
        super().__init__('EKF_odom_node')

def main(args=None):
    rclpy.init(args=args)
    node = EKF_odom_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
