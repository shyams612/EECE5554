from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port the VectorNav VN-100 is connected to'
    )

    imu_driver_node = Node(
        package='imu_driver',
        executable='imu_driver',
        name='imu_driver',
        output='screen',
        arguments=[LaunchConfiguration('port')]
    )

    return LaunchDescription([
        port_arg,
        imu_driver_node,
    ])