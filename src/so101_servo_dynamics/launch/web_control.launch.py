from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for SO101 STS3215 servo driver'
    )

    baud_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='1000000',
        description='Baudrate (default: 1000000)'
    )

    # 1. Hardware connection & real-time servo driver
    hardware_node = Node(
        package='so101_servo_dynamics',
        executable='hardware_connection',
        name='so101_hardware_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'publish_rate_hz': 30.0,
            'enable_sim_fallback': True
        }]
    )

    # 2. Web control & data recording bridge
    web_bridge_node = Node(
        package='so101_servo_dynamics',
        executable='web_control_bridge',
        name='so101_web_bridge_node',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        hardware_node,
        web_bridge_node
    ])
