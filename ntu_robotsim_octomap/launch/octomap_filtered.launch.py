from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='OctoMap resolution in meters'
    )

    octomap_node = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[{
            # Map resolution
            'resolution': LaunchConfiguration('resolution'),

            # Map reference frame (must exist in TF)
            'frame_id': 'odom',

            # Robot base frame (must exist in TF)
            'base_frame_id': 'base_link',

            # Ground filtering parameters
            'filter_ground': True,
            'ground_filter.distance': 0.04,
            'ground_filter.angle': 0.15,
            'ground_filter.plane_distance': 0.07,

            # Sensor model
            'sensor_model.max_range': 6.0
        }],
        remappings=[
            ('cloud_in', '/rgbd_camera/points'),
            ('projected_map', '/projected_map')
        ]
    )

    return LaunchDescription([
        resolution_arg,
        octomap_node
    ])
