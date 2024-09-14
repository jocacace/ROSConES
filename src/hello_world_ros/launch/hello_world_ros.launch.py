import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()


    sdf_file_path = os.path.join(
        FindPackageShare('hello_world_ros').find('hello_world_ros'),
        'world',
        'hello_world_ros_plugin.sdf'
    )
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {sdf_file_path}'
        }.items()
    )

    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/hello_world_ros")    
    
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    
    ld.add_action( ign_resource_path)
    ld.add_action( gazebo_node)

    return ld