from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    package_name = 'ntu_robotsim_octomap'

    # Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'maze.launch.py'
            ])
        )
    )

    # Robot spawn
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'single_robot_sim.launch.py'
            ])
        )
    )

    # TF bridge
    odom_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odom_to_tf_ros2'),
                'launch',
                'odom_to_tf.launch.py'
            ])
        )
    )

    # OctoMap (ground filtered)
    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'octomap_filtered.launch.py'
            ])
        )
    )

    # Nav2
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
            'params_file': PathJoinSubstitution([
                FindPackageShare(package_name),
                'config',
                'nav2_octomap_full.yaml'
            ])
        }.items()
    )

    # RViz (Nav2 default config)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        )
    )

    return LaunchDescription([
        gazebo,
        robot_spawn,
        odom_tf,
        octomap,
        nav2,
        rviz
    ])
