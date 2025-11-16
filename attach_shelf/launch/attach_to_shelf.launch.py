from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.3',
        description='Distance to obstacle in meters at which robot stops'
    )

    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='-90',
        description='Degrees to rotate after stopping (negative = right turn)'
    )

    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='false',
        description='Whether to perform final approach and attach to shelf'
    )

    # Get launch configurations
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')
    final_approach = LaunchConfiguration('final_approach')

    # Approach service server node
    approach_service_server_node = Node(
        package='attach_shelf',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server'
    )

    # Pre-approach v2 node
    pre_approach_v2_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2',
        parameters=[{
            'obstacle': obstacle,
            'degrees': degrees,
            'final_approach': final_approach
        }]
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        approach_service_server_node,
        pre_approach_v2_node
    ])
