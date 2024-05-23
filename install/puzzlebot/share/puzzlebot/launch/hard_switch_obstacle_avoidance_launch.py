import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

package_name = 'puzzlebot'
package_share = get_package_share_directory(package_name)
wheel_transforms_config = os.path.join(package_share, 'config', 'wheel_transforms_publisher.yaml')
autonomous_robot_constants_config = os.path.join(package_share, 'config', 'autonomous_robot_constants.yaml')

def generate_launch_description() -> LaunchDescription:
    urdf_file_name = 'urdf/Puzzlebot.urdf'
    urdf = os.path.join(package_share, urdf_file_name)
    assert os.path.exists(urdf), "URDF file not found at %s" % urdf
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_desc}],
        arguments = [urdf],
    )

    wheel_transforms_publisher = Node(
        package=package_name,
        executable='wheel_transforms_publisher',
        name='wheel_transforms_publisher',
        output='screen',
        parameters=[wheel_transforms_config, {'use_sim_time': True}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf'],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'Puzzlebot', 
                   '-file', urdf,
                   '-x', '0', 
                   '-y', '0', 
                   '-z', '0.04'],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/Puzzlebot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/Puzzlebot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/model/Puzzlebot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/raw_lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/model/Puzzlebot/odometry', '/odometry'),
                    ('/model/Puzzlebot/cmd_vel', '/cmd_vel'),]
    )

    hard_switch_control = Node(
        package=package_name,
        executable='hard_switch_control',
        name='hard_switch_control',
        output='screen',
        parameters=[autonomous_robot_constants_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        wheel_transforms_publisher,
        rviz2,
        gazebo,
        spawn,
        gz_bridge,
        hard_switch_control,
    ])
