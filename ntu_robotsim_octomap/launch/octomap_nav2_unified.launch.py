from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('ntu_robotsim_octomap')

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='OctoMap resolution in meters'
    )

    maze = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'maze.launch.py'])
        )
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'single_robot_sim.launch.py'])
        )
    )

    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'octomap_filtered.launch.py'])
        ),
        launch_arguments={'resolution': LaunchConfiguration('resolution')}.items()
    )

    odom_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odom_to_tf_ros2'),
                'launch',
                'odom_to_tf.launch.py'
            ])
        )
    )

    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'use_lifecycle_mgr': 'True',
            'params_file': PathJoinSubstitution([
                pkg_share,
                'config',
                'nav2_octomap_full.yaml'
            ])
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'rviz_config': PathJoinSubstitution([
                pkg_share,
                'config',
                'octomap_3d.rviz'
            ])
        }.items()
    )

    return LaunchDescription([
        resolution_arg,
        maze,
        robot,
        octomap,
        odom_tf,
        tf_map_odom,
        nav2,
        rviz
    ])
