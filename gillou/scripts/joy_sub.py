#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_joy = self.create_subscription(
            Joy,
            '/joy',
            self.listener_joy_callback,
            10)
        self.subscription_joy  # prevent unused variable warning
        self.vit_x = 0
        self.rot_z = 0
        # self.publisher_ = self.create_publisher(Vector3, 'position_robot', 10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_joy = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("ccccccccc")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.vit_x
        msg.angular.z = self.rot_z
        self.publisher_joy.publish(msg)
        print('a')

    def listener_joy_callback(self, msg):
        self.vit_x = msg.axes[1]
        self.rot_z = msg.axes[0]
        print('bbb')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
