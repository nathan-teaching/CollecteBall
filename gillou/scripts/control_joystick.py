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
    print('received', key, "a", key.value, "a", key.keyname, "a", key.keytype, "a", key.number)
    rclpy.spin_once(publisher, timeout_sec=0)
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
        msg.angular.z = 3.
    elif key.value == Key.HAT_RIGHT:
        #do something
        msg.linear.x = 0.5
        msg.angular.z = -3.
    elif key.value == Key.HAT_CENTERED :
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.angular.z = 0.
    if key.keyname == "Axis 2":
        print("avant")
        msg.linear.x = 1.
    if key.keyname == "Axis 5":
        msg.linear.x = -1.
    if key.keyname in ["Axis 0", "Axis 1", "-Axis 0", "-Axis 1"]:
        print(key.keyname)
        # msg.linear.x = 1.
        msg.angular.z = 30 * np.abs(key.value)
    if key.keyname in ["Axis 3", "Axis 4", "-Axis 3", "-Axis 4"]:
        # msg.linear.x = -1.
        msg.angular.z = 30 * -np.abs(key.value)
    if key.keyname in ["Button 6", "Button 7"]:
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.angular.z = 0.
    publisher.publisher_.publish(msg)
    rclpy.spin_once(publisher, timeout_sec=0)

if __name__ == "__main__" :
    rclpy.init(args=None)
    publisher = MinimalPublisher()
    run_event_loop(print_add, print_remove, key_received)
