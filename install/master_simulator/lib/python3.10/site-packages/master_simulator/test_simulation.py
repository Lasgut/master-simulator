#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class ForcePublisher(Node):
    def __init__(self):
        super().__init__('force_publisher')
        self.force_publisher = self.create_publisher(Wrench, 'forces_topic', 10)

        msg = Wrench()
        msg.force.x = 0.1
        self.force_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    force_publisher = ForcePublisher()
    rclpy.spin(force_publisher)
    force_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()