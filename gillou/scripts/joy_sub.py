#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription_joy = self.create_subscription(
            Joy,
            '/joy',
            self.listener_joy_callback,
            10)
        self.subscription_joy # prevent unused variable warning
        self.vit_x = 0
        self.rot_z = 0
        # self.publisher_ = self.create_publisher(Vector3, 'position_robot', 10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_joy = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.published = True
        # print("ccccccccc")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.vit_x)
        msg.angular.z = float(self.rot_z)
        self.publisher_joy.publish(msg)
        self.published = True
        # print('a')

    def listener_joy_callback(self, msg):
        if self.published :
            self.vit_x = 0.9 * self.vit_x + 0.1 * msg.axes[1] 
            self.rot_z = 0.9 * self.rot_z + 0.1 * msg.axes[0] * np.sign(msg.axes[1])
            self.published = False
        # print(self.vit_x, self.rot_z)
        # print("bbb")

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
