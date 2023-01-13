#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int16MultiArray


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int16MultiArray(),
            "positions_balles",
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        lis_interm = []
        for i in range(0, len(msg.data), 3):
            lis_interm.append([msg.data[i], msg.data[i + 1], msg.data[i + 2]])
        lis_balls = lis_interm
        print(lis_balls)




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


# TODO define a thin square with coordinates given and radius given and find if a ball is in or not
def ball_in_traj(b, x_robot, y_robot, x_dest, y_dest, radius):
    """
    input: ball [x,y,time], robot coordinates, destination coordinates, radius to search
    output: True if the ball is in the traj, False otherwise
    """
    pass


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
