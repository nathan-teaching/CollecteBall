#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
# import matplotlib.pyplot as plt


def det_lis_balls(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # on effectue un masque avec les valeurs ci-dessous recuperee sur internet
    # pour ne garder que les lignes jaunes
    lower = np.array([70, 220, 100], dtype=np.uint8)
    upper = np.array([150, 255, 255], dtype=np.uint8)
    seg0 = cv2.inRange(hsv, lower, upper)
    cv2.imshow("detected balls", seg0)
    cv2.waitKey(1)
    lis_pixels = []
    coord_x, coord_y = np.array(np.where(seg0 == 255))
    for i in range(len(coord_x)):
        coord = [coord_x[i], coord_y[i]]
        already_in_lis = False
        for pix in lis_pixels:
            if np.abs(coord[0] - pix[0]) <= 10 or\
              np.abs(coord[1] - pix[1]) <= 10:
                already_in_lis = True
        if not (already_in_lis):
            lis_pixels.append(coord)
    return lis_pixels


def detect_zone(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # on effectue un masque avec les valeurs ci-dessous recuperee sur internet
    # pour ne garder que les lignes jaunes
    lower = np.array([100, 40, 90], dtype=np.uint8)
    upper = np.array([110, 255, 255], dtype=np.uint8)
    seg0 = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    seg0 = cv2.morphologyEx(seg0, cv2.MORPH_OPEN, kernel)
    # cv2.imshow("test", seg0)
    # cv2.waitKey(1)
    # plt.imshow(seg0)
    # plt.show()
    coord_x, coord_y = np.array(np.where(seg0 == 255))
    # print("pixels des safe zones : ", np.array([coord_x, coord_y]))
    safe_l_x, safe_l_y = [], []
    safe_r_x, safe_r_y = [], []
    for i in range(len(coord_y)):
        if coord_y[i] <= 650:
            safe_l_x.append(coord_x[i])
            safe_l_y.append(coord_y[i])
        else:
            safe_r_x.append(coord_x[i])
            safe_r_y.append(coord_y[i])
    safe_l_x, safe_l_y = np.array(safe_l_x), np.array(safe_l_y)
    safe_r_x, safe_r_y = np.array(safe_r_x), np.array(safe_r_y)
    coord_l_center = [np.mean(safe_l_x), np.mean(safe_l_y)]
    coord_r_center = [np.mean(safe_r_x), np.mean(safe_r_y)]
    print("coords centres : ", coord_l_center, coord_r_center)
    coord_l_entry = [np.max(safe_l_x), np.max(safe_l_y)]
    coord_r_entry = [np.min(safe_r_x), np.min(safe_r_y)]
    print("coords entrees : ", coord_l_entry, coord_r_entry)


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
        for i in range(len(self.lis_balls)):
            ball = self.lis_balls[i]
            data.append(ball[0])
            data.append(ball[1])
            data.append(i)
        # print(data)
        msg.data = [int(data[i]) for i in range(len(data))]
        # print(msg.data)
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # plt.imshow(current_frame)
        # plt.show()
        # det_safe_zone(current_frame)
        detect_zone(current_frame)
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
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or\
                      np.abs(new_ball[1] - old_ball[0][1]) <= 10:
                        paired = True
                if not paired:
                    coords_new_ball.append(new_ball)
            for old_ball in self.lis_balls:
                paired = False
                for new_ball in new_lis:
                    if np.abs(new_ball[0] - old_ball[0][0]) <= 10 or\
                      np.abs(new_ball[1] - old_ball[0][1]) <= 10:
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
                if np.abs(new_ball[0] - ball[0][0]) <= 10 or\
                  np.abs(new_ball[1] - ball[0][1]) <= 10:
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
