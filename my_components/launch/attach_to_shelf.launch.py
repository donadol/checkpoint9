import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with PreApproach and AttachServer components.

    This launch file starts a container named 'my_container' and loads:
    - PreApproach component with name 'pre_approach'
    - AttachServer component with name 'attach_server'

    Note: RViz cannot be launched with components. To visualize:
    rviz2 -d ~/ros2_ws/src/checkpoint9/my_components/rviz/config.rviz
    """

    # Component container with PreApproach and AttachServer components
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach'),
            ComposableNode(
                package='my_components',
                plugin='my_components::AttachServer',
                name='attach_server'),
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        container
    ])
