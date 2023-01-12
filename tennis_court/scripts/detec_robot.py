#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
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

    def listener_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # Display image
        cv2.imshow("camera", current_frame)
        #step = msg.step
        #rows = msg.height
        #image = msg.data
        #print(step, rows)
        #print(len(image))
        #img1 = np.zeros((step, rows))
        #for i in range(rows):
        #    for j in range(rows):
        #        #print(i, j)
        #        img1[i,j] = image[step*i+j]
        #print(img1)
        #hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        ## on effectue un masque avec les valeurs ci-dessous recuperee sur internet
        ## pour ne garder que les lignes jaunes
        #lower = np.array([20, 70, 100], dtype=np.uint8)
        #upper = np.array([30, 255, 255], dtype=np.uint8)
        #seg0 = cv2.inRange(hsv, lower, upper
        # on affiche l'image
        cv2.waitKey(0)

        sum = 0  # la somme des positions en x des pixels blancs
        cnt = 0  # le nombre de pixels blancs
        return current_frame



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