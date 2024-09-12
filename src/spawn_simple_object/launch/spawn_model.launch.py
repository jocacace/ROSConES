from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import  Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()
    xacro_path = 'urdf/cube.urdf.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('spawn_simple_object'),	
        xacro_path
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )

    # Spawn
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'cube',    
                    '-x', '1',
                    '-y', '1',
                    '-z', '0.5',
                    '-r', '0',
                    '-p', '0',
                    '-Y', '3.14',
                    '-topic', '/robot_description'],
                 output='screen')

    
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
   

    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )
    ld.add_action( ignition_gazebo_node )
    return ld