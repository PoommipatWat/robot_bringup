from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ekf_config_path = PathJoinSubstitution([FindPackageShare('robot_bringup'), 'config', 'ekf.yaml'])


    return LaunchDescription([
        Node(
            package='robot_bringup',
            executable='robot_bringup',
            name='bringup',
            output='screen'
        ),
        Node(
            package='robot_bringup',
            executable='robot_odom',
            name='odom',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0.065', '0', '0.02','3.14159265', '0', '0','base_link','laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_pub_laser',
            arguments=['0.0', '0', '0.0','0.0', '0', '0','base_link','imu_link'],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/odometry/filtered', '/odom')]
        ),
    ])