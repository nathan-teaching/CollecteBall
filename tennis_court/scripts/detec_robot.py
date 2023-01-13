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
        self.lis_balls = []

    def listener_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # print(detec_ball.det_lis_balls(current_frame))
        self.update_lis_balls(detec_ball.det_lis_balls(current_frame))
        print(self.lis_balls)
        # # Display image
        # cv2.imshow("camera", current_frame)

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

        # cv2.waitKey(0)

        sum = 0  # la somme des positions en x des pixels blancs
        cnt = 0  # le nombre de pixels blancs
        return current_frame
    
    def update_lis_balls(self, new_lis):
        if len(self.lis_balls) < len(new_lis):
            self.add_balls(new_lis)
        else :
            coords_old_ball = []
            coords_new_ball = []
            for new_ball in new_lis :
                paired = False
                for old_ball in self.lis_balls :
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or np.abs(new_ball[1] - old_ball[0][1]) <= 10 :
                        paired = True
                if not paired :
                    coords_new_ball.append(new_ball)
            for old_ball in self.lis_balls :
                paired = False
                for new_ball in new_lis :
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or np.abs(new_ball[1] - old_ball[0][1]) <= 10 :
                        paired = True
                if not paired :
                    coords_old_ball.append(old_ball)
            if len(coords_new_ball) == len(coords_old_ball) : 
                for i in range(len(coords_old_ball)) :
                    coord = coords_old_ball[i]
                    self.lis_balls[self.lis_balls.index(coord)] = (coords_new_ball[i], coord[1])


    def add_balls(self, new_lis):
        for new_ball in new_lis :
            in_lis = False
            for ball in self.lis_balls :
                if np.abs(new_ball[0] - ball[0][0]) <= 10 or np.abs(new_ball[1] - ball[0][1]) <= 10 :
                    in_lis = True
            if not in_lis :
                self.lis_balls.append((new_ball, len(self.lis_balls)))



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