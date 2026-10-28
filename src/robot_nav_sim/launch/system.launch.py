from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    avoidance_enabled = ParameterValue(LaunchConfiguration('avoidance_enabled'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            'avoidance_enabled', default_value='true',
            description='Let the planner switch to detour goals around detected obstacles.'),
        Node(package='robot_nav_sim', executable='perception_node', output='screen'),
        Node(package='robot_nav_sim', executable='planner_node', output='screen',
             parameters=[{'avoidance_enabled': avoidance_enabled}]),
        Node(package='robot_nav_controller', executable='driver_node', output='screen'),
    ])
