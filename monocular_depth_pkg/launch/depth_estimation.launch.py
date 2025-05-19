from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start our depth node with three parameters
        Node(
            package='monocular_depth_pkg',
            executable='depth_node',
            name='mono_depth',
            output='screen',
            parameters=[{
                'camera_height': 1.55,  # change if your CARLA camera is higher
                'fy': 630.0,            # get from CARLA sensor blueprint
                'yh': 180.0             # horizon row (pixels)
            }]
        )
    ])

