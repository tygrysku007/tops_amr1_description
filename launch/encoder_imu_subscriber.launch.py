from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tops_amr1_description',
            executable='encoder_subscriber',
            name='encoder_subscriber_node',  # Add unique names for each node
            output='screen',
        ),

        Node(
            package='tops_amr1_description',
            executable='imu_subscriber',
            name='imu_subscriber_node',  # Add unique names for each node
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        )
    ])

