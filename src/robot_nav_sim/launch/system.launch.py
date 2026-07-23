from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_nav_sim', executable='perception_node', output='screen'),
        Node(package='robot_nav_sim', executable='planner_node', output='screen'),
        Node(package='robot_nav_sim', executable='robot_driver', output='screen'),
    ])