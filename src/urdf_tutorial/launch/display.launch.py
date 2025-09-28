from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare 'model' launch argument with a default value (relative URDF path)
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='urdf/simple-link.urdf',
        description='Relative path to robot URDF file'
    )

    model_path = LaunchConfiguration('model')

    return LaunchDescription([
        # Launch argument
        model_arg,

        # Launch robot_state_publisher with robot_description parameter set from model arg
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': model_path}]
        ),

        # Launch joint_state_publisher_gui node for interactive joint control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d']  # optionally specify rviz config here
        ),
    ])
