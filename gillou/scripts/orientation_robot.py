#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/zenith_camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.orientation_robot = 0.0
        self.publisher_ = self.create_publisher(
            Float32, 'orientation_robot', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription2 = self.create_subscription(
            Vector3, 
            "/position_robot", 
            self.listener_pos_rob_callback, 
            10)
        self.subscription2
        self.position_robot = (0, 0)

    def timer_callback(self):
        msg = Float32()
        msg.data = float(self.orientation_robot)
        self.publisher_.publish(msg)


    def listener_pos_rob_callback(self, msg):
        pos_x = msg.x
        pos_y = msg.y
        self.position_robot = (pos_x, pos_y)
        print(self.position_robot)

    def listener_callback(self, msg):

        current_frame = self.br.imgmsg_to_cv2(msg)
        scale = 0.3
        dim = [int(current_frame.shape[1]*scale), int(current_frame.shape[0]*scale)]
        current_frame = cv2.resize(current_frame, dim, interpolation=cv2.INTER_AREA)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # filtre pour détecter la pelle
        hsv2 = cv2.cvtColor(current_frame, cv2.COLOR_RGB2HSV)
        lower2 = np.array([0, 180, 0], dtype=np.uint8)
        upper2 = np.array([80, 255, 150], dtype=np.uint8)
        seg1 = cv2.inRange(hsv2, lower2, upper2)
        # cv2.imshow("orientation", seg1)
        pixel_blanc_x = []
        pixel_blanc_y = []
        for i in range(len(seg1)):
            for j in range(len(seg1[0])):
                if seg1[i, j] == 255:
                    pixel_blanc_x.append(i)
                    pixel_blanc_y.append(j)
        pos_x_pelle = int((np.sum(pixel_blanc_x)/len(pixel_blanc_x))/scale)
        pos_y_pelle = int((np.sum(pixel_blanc_y)/len(pixel_blanc_y))/scale)
        position_pelle = (pos_x_pelle, pos_y_pelle)
        # self.get_logger().info("On est avant angle")
        angle = self.angle(self.position_robot, position_pelle)
        # self.get_logger().info("On est après angle")
        self.orientation_robot = angle

    def angle(self, pos_rob, pos_pelle):
        vect = np.array([[pos_pelle[0]-pos_rob[0]], [pos_pelle[1]-pos_rob[1]]])
        angle = np.arccos(vect[0, 0]/np.linalg.norm(vect))
        # xr, yr = pos_rob
        # xp, yp = pos_pelle

        # # self.get_logger().info('xp: "%f"' % xp)
        # angle = np.arctan((yp - yr)/(xp - xr))
        self.get_logger().info('angle: "%f"' % angle)

        return angle


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
