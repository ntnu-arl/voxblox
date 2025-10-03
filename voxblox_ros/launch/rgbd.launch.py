from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.20',
        description='TSDF voxel size'
    )
    
    # Get launch configurations
    voxel_size = LaunchConfiguration('voxel_size')
    
    # Get package share directory
    voxblox_ros_share = FindPackageShare('voxblox_ros')
    
    # Path to config file (optional - comment out if you don't have this file)
    config_file = PathJoinSubstitution([
        voxblox_ros_share,
        'cfg',
        'rgbd_dataset.yaml'
    ])
    
    # Voxblox node
    voxblox_node = Node(
        package='voxblox_ros',
        executable='tsdf_server',
        name='voxblox_node',
        output='screen',
        parameters=[
            # config_file,  # Uncomment if you have a config file
            {
                'tsdf_voxel_size': 0.2,
                'tsdf_voxels_per_side': 16,
                'voxel_carving_enabled': True,
                'color_mode': 'color',
                'use_tf_transforms': True,
                'update_mesh_every_n_sec': 1.0,
                'verbose': True,
                'min_time_between_msgs_sec': 0.2,
                'max_ray_length_m': 2.0,
                'world_frame': 'world',
                'publish_pointclouds': True
                # 'mesh_filename': '/home/arl/workspaces2/arl_autonomy_ws/voxblox_mesh.ply'  # Uncomment to save mesh
            }
        ],
        remappings=[
            ('pointcloud', '/camera/depth/points_fixed')
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        voxel_size_arg,
        voxblox_node,
    ])