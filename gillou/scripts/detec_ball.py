#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge


def det_lis_balls(img, H=60, tolerance=30):
    # safe zone : H=30
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    img_comp = img[:, :, 0]
    coord_x, coord_y = np.array(np.where(np.abs(img_comp - H) <= tolerance))
    blank = np.zeros((img.shape))

    lis_pixels = []
    if len(coord_x) == 0:
        return lis_pixels
    else:
        for i in range(len(coord_x)):
            coord = (coord_x[i], coord_y[i])
            already_in_lis = False
            for pix in lis_pixels:
                if np.abs(coord[0] - pix[0]) <= 10 or\
                 np.abs(coord[1] - pix[1]) <= 10:
                    already_in_lis = True
            if not (already_in_lis):
                lis_pixels.append(coord)
        return lis_pixels


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

        self.publisher_ =\
            self.create_publisher(Int16MultiArray, 'positions_balles', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # print("coucou")
        msg = Int16MultiArray()
        data = []
        for ball in self.lis_balls:
            data.append(ball[0][0])
            data.append(ball[0][1])
            data.append(ball[1])
        # print(data)
        msg.data = [int(data[i]) for i in range(len(data))]
        print(msg.data)
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        self.update_lis_balls(det_lis_balls(current_frame))
        # print(self.lis_balls)

    def update_lis_balls(self, new_lis):
        if len(self.lis_balls) < len(new_lis):
            self.add_balls(new_lis)
        else:
            coords_old_ball = []
            coords_new_ball = []
            for new_ball in new_lis:
                paired = False
                for old_ball in self.lis_balls:
<<<<<<< HEAD
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or np.abs(new_ball[1] - old_ball[0][1]) <= 10:
=======
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or\
                      np.abs(new_ball[1] - old_ball[0][1]) <= 10:
>>>>>>> dd0196abb6ed648945ad048d14d8572b6851cdfd
                        paired = True
                if not paired:
                    coords_new_ball.append(new_ball)
            for old_ball in self.lis_balls:
                paired = False
                for new_ball in new_lis:
<<<<<<< HEAD
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or np.abs(new_ball[1] - old_ball[0][1]) <= 10:
=======
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or\
                      np.abs(new_ball[1] - old_ball[0][1]) <= 10:
>>>>>>> dd0196abb6ed648945ad048d14d8572b6851cdfd
                        paired = True
                if not paired:
                    coords_old_ball.append(old_ball)
            if len(coords_new_ball) == len(coords_old_ball):
                for i in range(len(coords_old_ball)):
                    coord = coords_old_ball[i]
                    self.lis_balls[self.lis_balls.index(coord)] =\
                        (coords_new_ball[i], coord[1])

    def add_balls(self, new_lis):
        for new_ball in new_lis:
            in_lis = False
            for ball in self.lis_balls:
<<<<<<< HEAD
                if np.abs(new_ball[0] - ball[0][0]) <= 10 or np.abs(new_ball[1] - ball[0][1]) <= 10:
=======
                if np.abs(new_ball[0] - ball[0][0]) <= 10 or\
                  np.abs(new_ball[1] - ball[0][1]) <= 10:
>>>>>>> dd0196abb6ed648945ad048d14d8572b6851cdfd
                    in_lis = True
            if not in_lis:
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
