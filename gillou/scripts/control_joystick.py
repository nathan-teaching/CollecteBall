#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from pyjoystick.sdl2 import Key, Joystick, run_event_loop

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, 'demo/cmd_vel', 10)



def print_add(joy):
    print('Added', joy)

def print_remove(joy):
    print('Removed', joy)

def key_received(key):
    print('received', key)
    rclpy.spin_once(publisher, timeout_sec=0.01)
    msg = Twist()
    if key.value == Key.HAT_UP:
        #do something
        msg.linear.x = 1.
    elif key.value == Key.HAT_DOWN:
        #do something
        msg.linear.x = -1.
    if key.value == Key.HAT_LEFT:
        #do something
        msg.linear.x = 0.5
        msg.angular.z = 1.
    elif key.value == Key.HAT_RIGHT:
        #do something
        msg.linear.x = 0.5
        msg.angular.z = -1.
    elif key.value == Key.HAT_CENTERED :
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.angular.z = 0.
    elif key.value == Key.HAT_UPLEFT:
        #do something
        pass
    elif key.value == Key.HAT_DOWNLEFT:
        #do something
        pass
    elif key.value == Key.HAT_UPRIGHT:
        #do something
        pass
    elif key.value == Key.HAT_DOWNRIGHT:
        #do something
        pass
    publisher.publisher_.publish(msg)
    rclpy.spin_once(publisher, timeout_sec=0.01)

if __name__ == "__main__" :
    rclpy.init(args=None)
    publisher = MinimalPublisher()
    run_event_loop(print_add, print_remove, key_received)
