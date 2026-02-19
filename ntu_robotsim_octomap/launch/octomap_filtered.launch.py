from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    octomap_node = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[{
            # Map resolution
            'resolution': 0.05,

            # Map reference frame (must exist in TF)
            'frame_id': 'odom',

            # Robot base frame (must exist in TF)
            'base_frame': 'base_link',

            # Ground filtering parameters
            'filter_ground': True,
            'ground_filter_distance': 0.04,
            'ground_filter_angle': 0.15,
            'ground_filter_plane_distance': 0.07,

            # Sensor model
            'sensor_model.max_range': 6.0
        }],
        remappings=[
            ('cloud_in', '/rgbd_camera/points')
        ]
    )

    return LaunchDescription([
        octomap_node
    ])
