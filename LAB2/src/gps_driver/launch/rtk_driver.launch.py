from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the port argument
    port_arg = DeclareLaunchArgument(
        'port',
        description='Serial port for RTK device'
    )
    
    # Create the driver node with the port argument
    driver_node = Node(
        package='gps_driver',
        executable='rtk_driver',
        name='rtk_driver_node',
        arguments=['-p', LaunchConfiguration('port')],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        driver_node
    ])