from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='detector',
                plugin='detector_node::detector_node',
                name='detector',
                parameters=[{
                    'debug_mode': True
                }]
            )
        ],
        output='screen'
    )
    return LaunchDescription([container])