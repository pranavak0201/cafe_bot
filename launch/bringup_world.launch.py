from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_cafe = get_package_share_directory('cafe_bot')

    world_file = os.path.join(pkg_cafe, 'worlds', 'map1.world')
    x_pose = LaunchConfiguration('x_pose', default='0')
    y_pose = LaunchConfiguration('y_pose', default='0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),

        # Launch Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items()
        ),

        # Launch Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py'))
        ),

        # Spawn TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
        ),

        # Robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])
