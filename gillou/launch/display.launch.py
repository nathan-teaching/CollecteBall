import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import ExecuteProcess, RegisterEventHandler

from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='gillou').find('gillou')
    #pkg_share2=launch_ros.substitutions.FindPackageShare(package='tennis_court').find('tennis_court')
    default_model_path = os.path.join(pkg_share, 'src/description/gillou.urdf')
    #default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    tennis_court_share = get_package_share_directory("tennis_court")
    tennis_court_launch_file = os.path.join(tennis_court_share, "launch", "tennis_court.launch.py")
    tennis_court_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(tennis_court_launch_file)
    )

    #world_path=os.path.join(pkg_share2, 'worlds/court.world'),

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
        
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'gillou_bot', '-topic', 'robot_description','-x','1','-y','1', '-z', '1'],
        output='screen'
    )
#
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        tennis_court_launch,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        
        load_joint_trajectory_controller,
        load_joint_state_controller
     
      
    ])
        