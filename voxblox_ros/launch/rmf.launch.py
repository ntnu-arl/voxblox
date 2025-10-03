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
                'world_frame': 'world',
                'use_tf_transforms': True,
                'use_freespace_pointcloud': True,
                'tsdf_voxels_per_side': 16,
                'tsdf_voxel_size': 0.20,
                'max_ray_length_m': 15.0,
                'truncation_distance': 1.0,
                'voxel_carving_enabled': True,
                'use_sparsity_compensation_factor': True,
                'sparsity_compensation_factor': 100.0,
                'color_mode': 'color',
                'verbose': False,
                'update_mesh_every_n_sec': 2.0,
                'mesh_min_weight': 1e-4,
                'slice_level': 1.0,
                'method': 'fast',
                'integration_order_mode': 'sorted',
                'max_consecutive_ray_collisions': 0,
                'publish_slices': False,
                'publish_pointclouds': True,
                'allow_clear': True,
                'pointcloud_queue_size': 5,
                'min_time_between_msgs_sec': 0.0,
                'publish_tsdf_info': False,
                'publish_traversable': False,
                'traversability_radius': 1.0,
                'enable_icp': False,
                'icp_refine_roll_pitch': False,
                'accumulate_icp_corrections': True,
                'timestamp_tolerance_sec': 0.001,
                'publish_tsdf_map': True,
                'publish_esdf_map': True,
                'tsdf_surface_distance_threshold_factor': 3.0,
                'esdf_max_distance_m': 3.0,
                'clear_sphere_for_planning': False,
                'clear_sphere_radius': 0.8,
                'use_const_weight': False,
                'use_weight_dropoff': True,
                'use_symmetric_weight_drop_off': False,
                'max_weight': 50.0,
                'clearing_ray_weight_factor': 0.01,
                'weight_ray_by_range': False,
                'min_ray_length_m': 0.5,
                'occupancy_min_distance_voxel_size_factor': 1.0,
                'occupancy_distance_voxelsize_factor': 1.0,
            }
        ],
        remappings=[
            ('pointcloud', '/os_cloud_node/points')
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        voxel_size_arg,
        voxblox_node,
    ])