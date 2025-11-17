import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with PreApproach component.

    Note: RViz cannot be launched with components. To visualize:
    rviz2 -d ~/ros2_ws/src/checkpoint9/my_components/rviz/config.rviz
    """

    # Component container with PreApproach component
    # Task 1: PreApproach runs alone and should shutdown when complete
    container = ComposableNodeContainer(
        name='pre_approach_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach',
                parameters=[{'shutdown_on_complete': True}]),
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        container
    ])
