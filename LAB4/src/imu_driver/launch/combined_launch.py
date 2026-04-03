from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB0',
        description='Serial port the GPS puck is connected to'
    )

    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Serial port the VectorNav VN-100 is connected to'
    )

    gps_node = Node(
        package='gps_driver',
        executable='driver',
        name='gps_driver',
        output='screen',
        arguments=['-p', LaunchConfiguration('gps_port')]
    )

    imu_node = Node(
        package='imu_driver',
        executable='imu_driver',
        name='imu_driver',
        output='screen',
        arguments=[LaunchConfiguration('imu_port')]
    )

    return LaunchDescription([
        gps_port_arg,
        imu_port_arg,
        gps_node,
        imu_node,
    ])
