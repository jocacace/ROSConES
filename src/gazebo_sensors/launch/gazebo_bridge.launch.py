from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[                
                # Lidar (IGN -> ROS2)
                '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',                                
                # Camera (IGN -> ROS2)
                '/cube_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/cube_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                # depth (Point clouds)
                '/cube_depth/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                ],
        remappings=[
            ("/lidar", "/cube/lidar"),
        ],
        output='screen'
    )


    depth_cam_data2sensor_link = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='cam3Tolink',
                    output='log',
                    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'sensor_link', 'cube_with_sensors/base_link/d435_depth'])

    lidar2sensor_link = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='cam3Tolink',
                    output='log',
                    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'sensor_link', 'lidar_link'])

    return LaunchDescription([
        bridge,
        depth_cam_data2sensor_link,
        lidar2sensor_link
    ])