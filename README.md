# ROSCon_Es 2024 tutorial on Gazebo and ros2_control

This repo contains the source code to follow the ROSConES 2024 tutorial entitled: _Tutorial de Gazebo e integración con ros2_control_ made by Jonathan Cacace from __Eurecat__ robotics Unit
__Time schedule__ 19 de septiembre de 2024, 9.30-11.30 - 

## Getting started
### Requirements
The tutorial is designed for ROS2-Humble (LTS). You can install it on your system (ubuntu 22.04), or on a docker container. The repository contains the files needed to create the Docker image and instantiate the container. 

### Get the repository
You can download this repository whenever you want into your system. Assume to clone it into you home folder.
		
		$ cd ~ 
		$ git clone https://github.com/jocacace/ROSConES.git

### Build and run the image
	
		$ cd ROSConES/Docker 		
		$ docker build -t ros2roscon .		
		$ docker compose up # You will lose the control of this terminal tab
To attach a new terminal to this docker container use the _docker exec_ command
	
		$ docker exec -it ros2roscon_container bash

This last command opens the door for the docker container in the upper level of the Docker folder. This will be your ROS2 workspace. For example, here you can compile the workspace:

		$ source /opt/ros/humble/setup.bash
		$ colcon build --symlink-install
Now you can start compiling and running the examples!

## Starting Gazebo
The interface between the terminal and Gazebo is implemented via the _ign_ command

	The 'ign' command provides a command line interface to the ignition tools.
		ign <command> [options]
To start the simulator use:
		
		$ ign gazebo
This will open a welcome page where you can select to start a Gazebo scene, or an empty scene. This scene is quite unusable, it misses also the ground plan (try to include a new object in the environment, it falls down).
To open directly an empty scene use this command:

		$ ign gazebo empty.sdf

This is a better starting point. In this window, apart from the main stage, there are GUI elements: 
- World: all the elements
- Entity tree: the objects

## Gazebo topics
Internally, Gazebo shares information using topics, exactly like ROS 2. We will use this information to integrate ROS 2 and Gazebo later. Some commands are useful to understand the data active in the system:

	$ ign topic
		Options:
		  -h,--help                   Print this help message and exit
		  -v,--version                
		  -t,--topic TEXT             Name of a topic
		  -m,--msgtype TEXT           Type of message to publish
		  -d,--duration FLOAT Excludes: --num Duration (seconds) to run
		  -n,--num INT Excludes: --duration Number of messages to echo and then exit

		[Option Group: command]
			  Command to be executed
			  Options:
					-l,--list                   
				    -i,--info Needs: --topic    
				    -e,--echo                   		
				    -p,--pub TEXT Needs: --topic --msgtype

For example, to list the active topics:

		$ ign topic -l

## Example 1: Key Publisher from Gazebo
- Open Gazebo with an empty scene

		$ ign gazebo empty.sdf
- Add the Key Publisher plugin: click on the 3 vertical dots -> search and click on the Key Publisher plugin
- Start the simulation with the play button
- On a terminal, check for the current active topics:

		$ ign  topic -l
- Ask for the content of the topic: _/keyboard/keypress_

		$ ign topic -e -t /keyboard/keypress
- Now, using the keyboard on the Gazebo scene and check the output on the terminal. 

We have now the basic elements to use Gazebo with ROS2.

## Example 2: Spawn an object in the scene
Gazebo is a standalone software and it is agnostic to ROS 2. It support _SDF_ file as object model: a robot or a static object - _SDF_ stands for _Simulation Description Format_. You can find only different simulations with complex robots using directly the _SDF_. However, for a deeper integration with ROS we prefer the use of _URDF_ file format. 
Let's create a new package storing the model of a simple object and its launch file:

	$ ros2 pkg create spawn_simple_object --dependencies xacro
	$ cd spawn_simple_object
	$ mkdir launch
	$ mkdir urdf
	$ cd urdf && touch cube.urdf.xacro

The robot model is the following one. It is just one link, composed by the base link shaped as a cube.
```
<?xml version="1.0"?>
<robot name="cube" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
	      <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
	      <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="100"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>
</robot>
```
A launch file is used to start add the robot to the simulation:
- Import salient modules
```
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import  Command
from launch_ros.substitutions import FindPackageShare
```
- Retrieve the robot mode file (the xacro)
```
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
```
- Spwan the robot starting from the robot_description topic
```
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
``` 
- Start the Gazebo simulation
```
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
   

    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )
    ld.add_action( ignition_gazebo_node )
    return ld
```
Modify the CMakeLists.txt
```
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
```
Compile and source the workspace, before to start the simulation

    $ colcon build --symlink-install
    $ source install/setup.bash
    $ ros2 launch spawn_simple_object spawn_model.launch.py 

### Example 3: Sensors
Let's complicate our model by adding additional sensors. In particular, a standard camera, a depth camera and a lidar sensor. The sensors are added to the same cube. A input parameter from launch file define which sensor use in the simulation.
This package use the _realsense2-camera_ and _realsense2-description_ packages to simulate the depth sensor.



### Example 3: Bridge   
### Example 3: Plugin
### Example 3: Create a new robot model



