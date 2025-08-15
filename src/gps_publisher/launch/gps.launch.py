from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ------ Launch-time arguments ------ #
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='',
        description='Serial device path; empty -> auto-detect (/dev/ublox_gps* or /dev/ttyACM*)'
    )

    declare_baud = DeclareLaunchArgument(
        'baud',
        # default_value='115200',
        default_value='0',
        description='Serial baud rate for the GNSS receiver'
    )

    declare_rate = DeclareLaunchArgument(
        'rate_hz',
        default_value='10.0',
        description='Desired publish rate (Hz); node will downsample if the GPS is slower'
    )

    declare_uere = DeclareLaunchArgument(
        'uere_m',
        default_value='1.0',
        description='User-equivalent range error (metres) for DOP->sigma conversion'
    )


    gps_node = Node(
            package='gps_publisher',
            executable='gps_node',
            name='gps_publisher',
            output='screen',
            parameters=[{
            'port':    LaunchConfiguration('port'),
            'baud':    LaunchConfiguration('baud'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'uere_m':  LaunchConfiguration('uere_m'),
        }]
        )


    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_rate,
        declare_uere,
        gps_node,
    ])
