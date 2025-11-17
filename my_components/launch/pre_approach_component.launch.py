import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description with PreApproach component and RViz."""

    # Get RViz config file from my_components package
    pkg_my_components = get_package_share_directory('my_components')
    rviz_config_file = os.path.join(pkg_my_components, 'rviz', 'config.rviz')

    # Component container with PreApproach component
    container = ComposableNodeContainer(
        name='pre_approach_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach'),
        ],
        output='screen',
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Delay component container to allow TF tree to stabilize
    delayed_container = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[container]
    )

    return launch.LaunchDescription([
        rviz_node,
        delayed_container
    ])
