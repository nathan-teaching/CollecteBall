#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.node import Node
from sensor_msgs.msg import Image
import time



class Pelle_cmd(Node):

    def __init__(self):
        super().__init__('pelle_cmd')
        self.publisher1_ = self.create_publisher(
            Bool,
            '/bool_pelle',
            10)
        
        self.publisher_ = self.create_publisher(
            Twist,
            '/demo_pelle/cmd_vel',
            10)
        self.subscription = self.create_subscription(
            Bool,
            '/bool_pelle',
            self.listener_callback,
            10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        pass

    def listener_callback(self, msg):

        '''Monte ou baisse la pelle selon l'état de la pelle et la commande'''
        
        global memoire 
        cmd = Twist()
        test = Bool()
        test.data = msg.data
        
        if ((msg.data == True) and (memoire == False)):
            #baisser pelle
        
            cmd.linear.x = 0.1
            self.publisher_.publish(cmd)
            time.sleep(2.0)    
                
                
            memoire = True
                
            cmd.linear.x = 0.0

        elif ((msg.data == False) and (memoire == True)):
            #monter pelle
            
            cmd.linear.x = -0.1
            self.publisher_.publish(cmd)
            time.sleep(2.0)
            
            memoire = False
                
            cmd.linear.x = 0.0
        
        self.publisher_.publish(cmd)
        self.publisher1_.publish(test)
    


   


def main(args=None):
    rclpy.init(args=args)

    pelle_sub= Pelle_cmd()

    rclpy.spin(pelle_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pelle_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    memoire = False
    
    
    main()
