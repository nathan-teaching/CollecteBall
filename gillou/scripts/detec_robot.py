#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.position_robot = (0, 0)
        self.publisher_ = self.create_publisher(Vector3, 'position_robot', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Vector3()
        msg.x = float(self.position_robot[0])
        msg.y = float(self.position_robot[1])
        msg.z = 0.
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        try:
            current_frame = self.br.imgmsg_to_cv2(msg)
            current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

            hsv = cv2.cvtColor(current_frame, cv2.COLOR_RGB2HSV)
            # on effectue un masque avec les valeurs ci-dessous recuperee sur internet
            # pour ne garder que les lignes jaunes
            lower = np.array([100, 60, 150], dtype=np.uint8)
            upper = np.array([200, 90, 255], dtype=np.uint8)
            seg0 = cv2.inRange(hsv, lower, upper)
            pixel_blanc_x = []
            pixel_blanc_y = []
            for i in range(len(seg0)):
                for j in range(len(seg0[0])):
                    if seg0[i,j] == 255:
                        pixel_blanc_x.append(i)
                        pixel_blanc_y.append(j)
            pos_x = int(np.sum(pixel_blanc_x)/len(pixel_blanc_x))
            pos_y = int(np.sum(pixel_blanc_y)/len(pixel_blanc_y))
            self.position_robot = (pos_x, pos_y)
        except:
            self.get_logger().info("CRASH")
            pass


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
