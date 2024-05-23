import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'line_follower'
package_share = get_package_share_directory(package_name)
color_config = os.path.join(package_share, 'config', 'color_config.yaml')

def generate_launch_description() -> LaunchDescription:
    traffic_light_identification = Node(
        package=package_name,
        executable='traffic_light_identification',
        name='traffic_light_identification',
        output='screen',
        parameters=[color_config]
    )

    line_follower = Node(
        package=package_name,
        executable='line_follower',
        name='line_follower',
        output='screen',
    )


    return LaunchDescription([
        traffic_light_identification,
        line_follower,
    ])