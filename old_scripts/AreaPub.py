#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from abu_interfaces.msg import Area


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Area, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def Update(self):
        msg = Area()
        msg.level = self.i
        return msg
    def timer_callback(self):
        msg = self.Update()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
