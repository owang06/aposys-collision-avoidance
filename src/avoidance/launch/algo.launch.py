from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    openzen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openzen_driver'),
                'launch',
                'openzen_lpms.launch.py'
            ])
        ]),
    launch_arguments={'use_sim_time': 'false'}.items()
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gps_publisher'),
                'launch',
                'gps.launch.py'
            ])
        ]),
    launch_arguments={'use_sim_time': 'false'}.items()
    )

    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('radar_pub'),
                'launch',
                'publisher.launch.py'
            ])
        ])
    )

    drift_node = Node(
        package='imu_gps',
        executable='imu_gps_node',
        name='imu_gps',
        output='screen',
            parameters=[{'use_sim_time': False}]
    )

    algo_node = Node(
            package='avoidance',
            executable='algo_node',
            name='avoidance',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    
    viz_node = Node(
        package='avoidance',       # ← change this to match your package name
        executable='burst_viz',    # ← change if the executable is named differently
        name='burst_viz',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    radar_after_deps = RegisterEventHandler(
        OnProcessStart(
            target_action=algo_node,      # wait for this action to spawn
            on_start=[radar_launch],       # then start radar
        )
    )

    return LaunchDescription([
        openzen_launch,
        gps_launch,
        # drift_node,
        algo_node,
        radar_after_deps,
        viz_node
    ])
