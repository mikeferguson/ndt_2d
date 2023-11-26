#!/usr/bin/env python3

# Copyright 2023 Michael Ferguson
# All Rights Reserved

import sys

from ndt_2d.srv import Configure
import rclpy
from rclpy.node import Node

if __name__ == '__main__':
    rclpy.init()

    node = Node('ndt_save_map')
    client = node.create_client(Configure, 'configure')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for service...')

    req = Configure.Request()
    req.action = Configure.Request.SAVE_TO_FILE
    req.filename = sys.argv[1]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()
