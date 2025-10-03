from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='depth_image_proc_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', 'rgb/image'),
                    ('depth_registered/image_rect', 'depth_registered/image'),
                    ('points', 'depth_registered/points'),
                ],
            ),
        ]
    )

    return LaunchDescription([container])