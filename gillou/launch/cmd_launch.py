from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ball_order_node = Node(
        package='gillou',
        executable='ball_order.py',
    )

    orientation_robot_node = Node(
        package='gillou',
        executable='orientation_robot.py',
    )

    detec_ball_node = Node(
        package='gillou',
        executable='detec_ball.py',
    )

    detec_robot_node = Node(
        package='gillou',
        executable='detec_robot.py',
    )

    ld.add_action(orientation_robot_node)
    ld.add_action(detec_ball_node)
    ld.add_action(detec_robot_node)
    ld.add_action(ball_order_node)

    return ld
