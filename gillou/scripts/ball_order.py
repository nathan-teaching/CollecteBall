#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Vector3


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int16MultiArray(),
            "positions_balles",
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription2 = self.create_subscription(Vector3, "position_robot", self.listener_pos_rob_callback, 10)
        self.subscription2
        self.position_robot = (0,0)

    def listener_callback(self, msg):
        lis_interm = []
        for i in range(0, len(msg.data), 3):
            lis_interm.append([msg.data[i], msg.data[i + 1], msg.data[i + 2]])
        lis_balls = lis_interm
        print(lis_balls)
    
    def listener_pos_rob_callback(self, msg):
        pos_x = msg.x
        pos_y = msg.y
        self.position_robot = (pos_x, pos_y)
        print(self.position_robot)


# TODO find the correct coordinates
coords_zone = [[-100, -100], [100, 100]]
coords_net = [[-100, 0], [100, 0]]
net_sides = [[[0, 0], [0, 0]], [[0, 0], [0, 0]]]
lis_balls = []


def min_distance(x, y, list_coords):
    """
    input : x and y coordinates, a list of coordinates
    list_coords = [[x0, y0], [x1, y1], ....]
    output : x and y coordinates in list_coords for which the distance is minimal
    """
    d = []
    for c in list_coords:
        d.append([np.sqrt((c[0] - x)**2+(c[1] - y)**2), c])

    d.sort()
    return d[0][1][0], d[0][1][1]


def oldest(ball):
    """
    input: ball [x,y,time]
    output: time
    """
    return ball[2]


def path_balls(lis_balls, ball_obj, x_robot, y_robot, radius):
    # Careful ! lis_balls will be modified, therefore must not be the original one, rather a copy
    """determines the path for a robot, given a destination ball, in order to grab as many balls as possible in the way

    Args:
        lis_balls (int[3][]): list of the balls that could be in the trajectory of the robot
        ball_obj (int[3]): the destination ball that will be reached eventually
        x_robot (int): x coordinate of the robot
        y_robot (int): y coordinate of the robot
        radius (float): opening of the way. The bigger it is, the more likely the robot is to find a ball to grab in the way.

    Returns:
        int[2][]: the list of the coordinates of the balls that can be grabbed in the way
    """
    path = [(ball_obj[0], ball_obj[1])]
    for ball in lis_balls :
        if ball_in_traj(ball, x_robot, y_robot, ball_obj[0], ball_obj[1], radius) :
            lis_balls.remove(ball)
            path = path_balls(lis_balls, ball, x_robot, y_robot, 0.75 * radius) + path_balls(lis_balls, ball_obj, ball[0], ball[1], 0.75 * radius)
            break
    return path


def ball_in_traj(b, x_robot, y_robot, x_dest, y_dest, radius):
    """checks wether a ball b is within a rectangle formed by the segment between robot and dist and with a width 2 * radius

    Args:
        b (int[3]): the coordinates and the order of the ball
        x_robot (int): x coordinate of the robot
        y_robot (int): y coordinate of the robot
        x_dest (int): x coordinate of the objective ball
        y_dest (int): y coordinate of the objective ball
        radius (float): the semi-width of the rectangle

    Returns:
        bool: whether the ball is in the ractangle or not
    """
    complex_vect = np.array([x_dest - x_robot + j * (y_dest - x_dest)])
    angle = np.angle(complex_vect)[0]
    corners = [(x_robot + radius * np.sin(angle), y_robot - radius * np.cos(angle)), (x_robot - radius * np.sin(angle), y_robot + radius * np.cos(angle)),
                (x_dest - radius * np.sin(angle), y_dest + radius * np.cos(angle)), (x_dest + radius * np.sin(angle), y_dest - radius * np.cos(angle))] # 4 corners of the rectangle, turning clockwise
    bottom_right, bottom_left, top_left, top_right = corners

    coefs_first_vert_line = (0, 0)
    coefs_first_vert_line[0] = (bottom_left[1] - top_left[1]) / (bottom_left[0] - top_left[0])
    coefs_first_vert_line[1] = bottom_left[1] - coefs_first_line[0] * bottom_left[0]
    coefs_second_vert_line = (0, 0)
    coefs_second_vert_line[0] = (bottom_right[1] - top_right[1]) / (bottom_right[0] - top_right[0])
    coefs_second_vert_line[1] = bottom_right[1] - coefs_first_line[0] * bottom_right[0]
    coefs_first_horiz_line = (0, 0)
    coefs_first_horiz_line[0] = (bottom_left[1] - bottom_right[1]) / (bottom_left[0] - bottom_right[0])
    coefs_first_horiz_line[1] = bottom_left[1] - coefs_first_line[0] * bottom_left[0]
    coefs_second_horiz_line = (0, 0)
    coefs_second_horiz_line[0] = (top_right[1] - top_left[1]) / (top_right[0] - top_left[0])
    coefs_second_horiz_line[1] = top_right[1] - coefs_first_line[0] * top_right[0]

    if (ball[1] > coefs_first_vert_line[0] * ball[0] + coefs_first_vert_line[1] and ball[1] < coefs_second_vert_line[0] * ball[0] + coefs_second_vert_line[1])\
    or (ball[1] < coefs_first_vert_line[0] * ball[0] + coefs_first_vert_line[1] and ball[1] > coefs_second_vert_line[0] * ball[0] + coefs_second_vert_line[1]):
        if (ball[1] > coefs_first_horiz_line[0] * ball[0] + coefs_first_horiz_line[1] and ball[1] < coefs_second_horiz_line[0] * ball[0] + coefs_second_horiz_line[1])\
        or (ball[1] < coefs_first_horiz_line[0] * ball[0] + coefs_first_horiz_line[1] and ball[1] > coefs_second_horiz_line[0] * ball[0] + coefs_second_horiz_line[1]):
            return True 
    return False

def in_square(coords):
    """
    input: coordinates [x,y]
    output: Side of the net, "Left" or "Right"
    """
    left = [net_sides[0][0][0], net_sides[0][1][0]]
    # TODO verify if it is the x or y coordinates that change between sides
    right = [net_sides[1][0][0], net_sides[1][1][0]]

    if min(left) <= coords[0] <= max(left):
        return "Left"
    elif min(right) <= coords[0] <= max(right):
        return "Right"


def ball_to_fetch(ball_list, radius, objective="zone"):
    """
    input : list of balls with coordinates and time falling, a radius to search around, the objective 
    structure of ball_list : [[x,y,time], ...]
    objective can be "zone" or "ball"
    output : list of balls to fectch between the robot and the objective
    """
    x_robot, y_robot = 0, 0  # fetch the robot coordinates
    x_dest, y_dest = 0, 0

    if objective == "zone":
        x_dest, y_dest = min_distance(x_robot, y_robot, coords_zone)
    elif objective == "ball":
        ball_list.sort(key=oldest)

        x_dest, y_dest = ball_list[0][0], ball_list[0][1]
    else:
        return print("Please enter a correct objective")

    ball_to_fetch = []
    for b in ball_list:
        if (ball_in_traj(b, x_robot, y_robot, x_dest, y_dest, radius)):
            ball_to_fetch.append([b[0], b[1]])

    return ball_to_fetch

# TODO control the orientation of the robot and put it into a straight line motion


def straight_line(x, y, x_dest, y_dest):
    """
    robot goes in a straight line to the desired position
    input : coordinates of the robot and coordinates to go
    output : None, the robot goes to the desired position
    """
    pass


def goto(x_robot, y_robot, x_dest, y_dest):
    """
    input : coordinates of the robot and coordinates to go
    output : None, the robot goes to the desired position
    """
    if (in_square([x_robot, y_robot]) == in_square([x_dest, y_dest])):
        print("Robot is in the same side than the ball")
        straight_line(x_robot, y_robot, x_dest, y_dest)

    else:
        print("Robot need to change side")
        x_mid, y_mid = min_distance(x_robot, y_robot, coords_net)
        straight_line(x_robot, y_robot, x_mid, y_mid)
        straight_line(x_mid, y_mid, x_dest, y_dest)


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
