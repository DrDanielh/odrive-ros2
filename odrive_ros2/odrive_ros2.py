# Copyright (c) 2022 Dunder Mifflin, Inc.
# All rights reserved.

import rclpy
from rclpy.node import Node

class OdriveROS2(Node):
    def __init__(self):
        super().__init__('OdriveROS2')
        print("Initiated ODriveROS2 node.")


def main(args=None):
    rclpy.init(args=args)
    node = OdriveROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()