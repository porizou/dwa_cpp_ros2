import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='dwa_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='dwa_ros2',
                    plugin='dwa_ros2::ObstacleDetector',
                    name='obstacle_detector_node',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='dwa_ros2',
                    plugin='dwa_ros2::DWA',
                    name='dwa_node',
                    extra_arguments=[{'use_intra_process_comms': True}])
            ],
            output='both',
    )
    return launch.LaunchDescription([container])
