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

## Part 1: Getting started with modern Gazebo (Fortress (v6))
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

    $ sudo apt-get install ros-humble-realsense2-camera
    $ sudo apt-get install ros-humble-realsense2-description
    
Create the _gazebo_sensors_ package.
    $ ros2 pkg create gazebo_sensors --dependencies xacro realsense2_description
    
#### Define the xacro 
```
<?xml version="1.0"?>
    <robot name="cube" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:arg name="use_camera" default="false" />
        <xacro:arg name="use_depth"  default="false" />
        <xacro:arg name="use_lidar"  default="false" />
        <xacro:property name="use_camera" value="$(arg use_camera)" />    
        <xacro:property name="use_depth" value="$(arg use_depth)" />
        <xacro:property name="use_lidar" value="$(arg use_lidar)" />
```
- Define the base_link and the sensor lonk, associated with a fixed joint
```
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
    
    <link name="sensor_link">
            <inertial>
            <mass value="0.1"/>
            <origin xyz="0 0 0"/>
            <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.02"/>
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <material name="black"/>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.02"/>
            </geometry>
        </collision>
    </link>

    <joint name="base_to_sensor" type="fixed">
        <parent link="base_link"/>
        <child link="sensor_link"/>
        <origin xyz="0 0 0.2" rpy="0.0 0.0 3.1415"/>
    </joint>
```
- If the _use_camera_ param is true, include the camera sensor
```
    <xacro:if value="${use_camera}">                        
        <gazebo reference="sensor_link">
            <sensor name="cube_camera" type="camera">
                <always_on>1</always_on>
                <update_rate>30</update_rate>
                <visualize>1</visualize>
                <pose>0 0.0175 0.5125 0 -0 0</pose>
                <topic>/cube_camera/image_raw</topic>
                <camera name="camera">
                    <horizontal_fov>1.21126</horizontal_fov>
                    <image>
                        <width>640</width>
                        <height>480</height>
                        <format>RGB_INT8</format>
                    </image>
                    <clip>
                        <near>0.1</near>
                        <far>100</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0</mean>
                        <stddev>0.007</stddev>
                    </noise>
                </camera>
            </sensor>   
        </gazebo>
    </xacro:if>
```
- If the _use_depth_ camera is true, add the vision sensor. 
```
xacro:if value="${use_depth}">         
        <gazebo reference="base_link">
            <xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro"/>
            <xacro:sensor_d435 parent="base_link" use_nominal_extrinsics="true" add_plug="false" use_mesh="true">
                <origin xyz="0 0 0.5" rpy="0 0 0"/>
            </xacro:sensor_d435>

            <sensor name='d435_depth' type='depth_camera'>
                <ignition_frame_id>camera_link</ignition_frame_id>
                <always_on>1</always_on>
                <update_rate>90</update_rate>
                <visualize>1</visualize>
                <topic>/cube_depth/image_raw</topic>
                <pose>0 0.0175 0.0 0 -0 0</pose>
                <camera name='d435'>
                    <ignition_frame_id>camera_link</ignition_frame_id>
                    <horizontal_fov>1.48702</horizontal_fov>
                    <image>
                        <width>1280</width>
                        <height>720</height>
                    </image>
                    <clip>
                        <near>0.1</near>
                        <far>100</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0</mean>
                        <stddev>0.1</stddev>
                    </noise>
                </camera>
            </sensor> 
        </gazebo> 
    </xacro:if>
```
- If the _use_lidar_ param is true, add the lidar sensor
```
    <xacro:if value="${use_lidar}">         
        <gazebo reference="sensor_link">
            <sensor name="gpu_lidar" type="gpu_lidar">
                <pose>-0.064 0 0.121 0 0 0</pose>
                <topic>/cube/scan</topic>
                    <ignition_frame_id>base_scan</ignition_frame_id>
                    <update_rate>5</update_rate>
                    <ray>
                        <scan>
                            <horizontal>
                                <samples>360</samples>
                                <resolution>1</resolution>
                                <min_angle>0</min_angle>
                                <max_angle>6.28</max_angle>
                            </horizontal>
                        </scan>
                        <range>
                            <min>0.120000</min>
                            <max>20.0</max>
                            <resolution>0.015000</resolution>
                        </range>
                        <noise>
                            <type>gaussian</type>
                            <mean>0.0</mean>
                            <stddev>0.01</stddev>
                        </noise>
                    </ray>
                    <always_on>true</always_on>
                    <visualize>1</visualize>
                </sensor>
        </gazebo>
    </xacro:if>
```
- If at least one sensor is enabled, include the Gazebo-sensor plugin
```
    <xacro:if value="${use_camera or use_depth or use_lidar}"  >
        <gazebo>
            <plugin filename="libignition-gazebo-sensors-system.so" 
            name="ignition::gazebo::systems::Sensors"> 
                <ignition_frame_id>camera_link</ignition_frame_id>
                <render_engine>ogre</render_engine>
            </plugin>   
        </gazebo>
    </xacro:if>
```
#### Create the launch file~
```
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import  Command
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from pathlib import Path
```
- Get the robot description model, initializing it with the parameters. In the first case, the _use_camera_ is true.
```
def generate_launch_description():
    ld = LaunchDescription()
    xacro_path = 'urdf/cube_with_sensors.urdf.xacro'
 
    robot_description = xacro.process_file( Path(os.path.join( get_package_share_directory('gazebo_sensors'), xacro_path ) ), mappings={'use_camera': "True", 'use_depth': "False", 'use_lidar': "False"})
```
- Define the robot state publisher node. This node published the _robot_description_  topic.
```
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':robot_description.toxml()
        }]
    )
```
- Spawn the robot. It is used the robot_description topic
```
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'cube_with_sensors',    
                    '-x', '1',
                    '-y', '1',
                    '-z', '0.5',
                    '-r', '0',
                    '-p', '0',
                    '-Y', '3.14',
                    '-topic', '/robot_description'],
                 output='screen')
```
- Start Gazebo simulation.
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
#### Modify the CMakeLists.txt
```
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
```
Now you can compile the workspace and launch the simulation

    $ colcon build --symlink-install
    $ source install/setup.bash
    $ ros2 launch gazebo_sensors cube_with_sensors.launch.py

For each sensor a Gazebo plugin can be used to check the output of the sensors:
- Image display (image and depth)
- Visualize lidar
- Check the topic published on ignition
    
        $ ign topic -l

### Example 4: Bridge
The data stremed on Gazebo are not useful if we can not use them using ROS 2. To transfer the data from Gazebo to ROS 2 we must use the _ros_gz_bridge_.
The _ros_gz_bridge_ provides a network bridge which enables the exchange of messages between ROS 2 and Ignition Transport. Its support is limited to only certain message types. Not all of them are supported. The bridge can be used in two ways:
- Command line ROS program
- Launch file


    $ ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@GZ_MSG

The ros2 run ros_gz_bridge parameter_bridge command simply runs the parameter_bridge code from the ros_gz_bridge package. Then, we specify our topic /TOPIC over which the messages will be sent. The first @ symbol delimits the topic name from the message types. Following the first @ symbol is the ROS message type.

The ROS message type is followed by an @, [, or ] symbol where:

    @ is a bidirectional bridge.
    [ is a bridge from Gazebo to ROS.
    ] is a bridge from ROS to Ignition.

#### A simple case: Publish key strokes to ROS
- Start Gazebo and add the Key Publisher plugin: Aclick on the 3 vertical dots -> search and click on the Key Publisher plugin. 
- Get the Gazebo message type 
    
        $ ign topic -i -t /keyboard/keypress
            Publishers [Address, Message Type]:
            tcp://172.18.0.1:37005, ignition.msgs.Int32

- Start the bridge
    
        $ ros2 run ros_gz_bridge parameter_bridge /keyboard/keyprs@std_msgs/msg/Int32[ignition.msgs.Int32

- Listen the message on ROS

        $ ros2 topic echo /keyboard/keyprs
        
#### Bridge the Sensor data
Let's create a launch file to stream the Gazebo topics. 
1) Start the simulation and get the info about the topics we want to bridge on ROS 2

        $ ros2 launch gazebo_sensors cube_with_sensors.launch.py 

- /cube_camera/camera_info -> CameraInfo
        
        user@CL-JCACACE1:~/ros2_ws$ ign topic -i -t /cube_camera/camera_info
            Publishers [Address, Message Type]:
            tcp://172.18.0.1:42637, ignition.msgs.CameraInfo

- /cube_camera/image_raw -> Image

        user@CL-JCACACE1:~/ros2_ws$ ign topic -i -t /cube_camera/image_raw 
            Publishers [Address, Message Type]:
            tcp://172.18.0.1:42637, ignition.msgs.Image
            
- /cube_depth/image_raw/points -> PointCloudPacked            

        user@CL-JCACACE1:~/ros2_ws$ ign topic -i -t /cube_depth/image_raw/points
            Publishers [Address, Message Type]:
            tcp://172.18.0.1:42637, ignition.msgs.PointCloudPacked
            
            
- /lidar -> LaserScan
            
        user@CL-JCACACE1:~/ros2_ws$ ign topic -i -t /lidar                      
            Publishers [Address, Message Type]:
            tcp://172.18.0.1:42637, ignition.msgs.LaserScan

2) Create a launch file for the bridge: _gazebo_bridge.launch.py_ 

```
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
```
We can remap the topic to change the name of the data published by Gazebo, to adapt it to the rest of the ROS2 system.
```
        remappings=[
            ("/lidar", "/cube/lidar"),
        ],
        output='screen'
    )
```
We need to manually publish the tf from the sensor output to the base frame for the camera
```
 depth_cam_data2sensor_link = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='cam3Tolink',
                    output='log',
                    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'sensor_link', 'cube_with_sensors/base_link/d435_depth'])
```
And for the lidar
```
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
```
3) Use ROS 2 tool to visualize the data
- rqt: 

        $ sudo apt-get install ros-humble-rqt-*
        $ ros2 run rqt-image-view rqt-image-view
- rviz2
-- with this it is possible to visualize the lidar and the point clouds adding the proper plugin from the add plugin panel.

### Example 5: Differential drive robot
Let's start implementing our first mobile robot. This will also introduce us to the ros2 controllers. 
Create a robot with 2 passive wheels and 2 active wheels. 

    $ ros2 pkg create diff_drive_description --dependencies xacro
    $ mkdir diff_drive_description/urdf
    $ mkdir diff_drive_description/launch

1) Create a macro file and a main xacro file
-- Define a set of of macro to create the robot model, the inertia of the cylinder and others.
```
<?xml version="1.0"?>
<robot name="diff_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="M_PI" value="3.1415926535897931" />
    
    <material name="Black">
      <color rgba="0.0 0.0 0.0 1.0"/>
    </material>
    <material name="White">
      <color rgba="1.0 1.0 1.0 1.0"/>
    </material>  

    <xacro:macro name="cylinder_inertia" params="m r h">
        <inertia  ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                izz="${m*r*r/2}" /> 
    </xacro:macro>
```
-- Define the passive wheel joint. It is a fixed joint. We need of this wheel for a correct orientatoin of the base
```
    <xacro:macro name="passive_wheel_joint" params="name parent child *origin">
      <joint name="${name}" type="fixed" >
        <parent link="${parent}" />
        <child link="${child}" />
        <xacro:insert_block name="origin" />
      </joint>
    </xacro:macro>
```
-- The passive wheel is a spherical object
```
    <xacro:macro name="passive_wheel_link" params="name *origin">
        <link name="${name}">
            <visual>
                <xacro:insert_block name="origin" />
                <geometry>
                    <sphere radius="${passive_wheel_radius}" />
                </geometry>
                <material name="Black" />
            </visual>  
            <collision>
                <geometry>
                    <sphere radius="${passive_wheel_radius}" />
                </geometry>
                <origin xyz="0 0.02 0" rpy="${M_PI/2} 0 0" />
            </collision>      
            <inertial>
                <mass value="${passive_wheel_mass}" />
                    <origin xyz="0 0 0" />
                    <inertia  ixx="0.001" ixy="0.0" ixz="0.0"
                            iyy="0.001" iyz="0.0" 
                                izz="0.001" />
            </inertial>
        </link>
    </xacro:macro>
```
-- Define the active wheel. A cylinder with a continuous rotational joint
```
  <xacro:macro name="wheel" params="side parent translateX translateY"> 
    <link name="${side}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0  0 " /> 
        <geometry>
          <cylinder length="${wheel_height}" radius="${wheel_radius}" />
        </geometry>
              <material name="Black" />
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0 " />
        <geometry>
          <cylinder length="${wheel_height}" radius="${wheel_radius}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${wheel_mass}" />
        <origin xyz="0 0 0" />
				<inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" 
               izz="0.001" />
      </inertial>
    </link>


    <joint name="${side}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${side}_wheel"/>
      
        <origin xyz="0 ${translateY} 0" rpy="0 0 0" /> 
      <axis xyz="0 1 0" rpy="0 0 0" />
      <limit effort="100" velocity="100"/>
      <joint_properties damping="0.0" friction="0.0"/>
    </joint>

  </xacro:macro>
</robot>
```
-- We can create now the _xacro_ file. First include the macro file description
```
<?xml version="1.0"?>

<robot name="diff_robot" xmlns:xacro="http://ros.org/wiki/xacro">
<xacro:include filename="$(find diff_drive_description)/urdf/diff_drive_macro.xacro" /> 
```
-- Some parameters define the base shape
```
    <xacro:property name="base_radius" value="0.15" /> 
    <xacro:property name="passive_wheel_height" value="0.04" /> 
    <xacro:property name="passive_wheel_radius" value="0.025" /> 
    <xacro:property name="passive_wheel_mass" value="0.5" /> 
    <xacro:property name="wheel_radius" value="0.039" /> 
    <xacro:property name="wheel_height" value="0.02" />
    <xacro:property name="wheel_mass" value="2.5" /> 
```
-- Base link
```
	<link name="base_link">
		<inertial>
				<inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
			<mass value="5" />
			<origin xyz="0 0 0" />
			<cylinder_inertia  m="5" r="${base_radius}" h="0.02" />
		</inertial>    
		<visual>
			<origin xyz="0 0 0" rpy="0 0 0" />
			<geometry>
				<cylinder length="0.02" radius="${base_radius}" />
			</geometry>
			<material name="White" />
		</visual>  
		<collision>
			<origin xyz="0 0 0" rpy="0 0 0 " />
			<geometry>
				<cylinder length="0.02" radius="${base_radius}" />
			</geometry>
		</collision>     
	</link>
```
-- Add the passive wheels
```
	<xacro:passive_wheel_joint name="passive_wheel_front_joint"
		parent="base_link"
		child="passive_wheel_front_link">
		<origin xyz="0.115 0.0 0.007" rpy="${-M_PI/2} 0 0"/>
	</xacro:passive_wheel_joint>

	<xacro:passive_wheel_link name="passive_wheel_front_link">
			<origin xyz="0 0.02 0" rpy="${M_PI/2} 0 0" />
	</xacro:passive_wheel_link>
	
	<xacro:passive_wheel_joint 
		name="passive_wheel_back_joint"
		parent="base_link"
		child="passive_wheel_back_link">
		<origin xyz="-0.135 0.0 0.009" rpy="${-M_PI/2} 0 0"/>
	</xacro:passive_wheel_joint>
	
	<xacro:passive_wheel_link
		name="passive_wheel_back_link">
			<origin xyz="0.02 0.02 0 " rpy="${M_PI/2} 0 0" /> 
	</xacro:passive_wheel_link>
```
-- Add the wheels
```
	<xacro:wheel side="right" parent="base_link" translateX="0" translateY="-${base_radius}" />
	<xacro:wheel side="left" parent="base_link" translateX="0" translateY="${base_radius}" />
```
-- Add the lidar sensor
```
	<link name="lidar_link">
		 <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
			<box size="0.05 0.05 0.05" />
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
			<box size="0.05 0.05 0.05" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
	</link>
	<joint name='lidar_sensor_joint' type='fixed'>        
      <parent link="base_link"/>
      <child link="lidar_link"/>
    </joint>

```
-- Add the Gazebo-sensor 
```
	  <gazebo reference="lidar_link">
        <sensor name="lidar" type='gpu_lidar'>
            <pose>0 0 0.2 0 0 0</pose>
            <topic>lidar</topic>
            <update_rate>10</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>640</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.396263</min_angle>
                        <max_angle>1.396263</max_angle>
                    </horizontal>
                    <vertical>
                        <samples>16</samples>
                        <resolution>1</resolution>
                        <min_angle>-0.261799</min_angle>
                        <max_angle>0.261799</max_angle>
                    </vertical>
                </scan>
                <range>
                    <min>0.1</min>
                    <max>10.0</max>
                    <resolution>0.01</resolution>
                </range>
            </ray>
            <always_on>1</always_on>
            <visualize>true</visualize>
			<gz_frame_id>lidar_link</gz_frame_id>
         
        </sensor>
    </gazebo>
```
-- Add two plugnis:
--- __Sensor plugin__: to stream the Lidar data
--- __Differential drive plugin__: to control the base velocity, translating such velocity to wheels velocity
```
	<gazebo>
        <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
            <left_joint>left_wheel_joint</left_joint>
            <right_joint>right_wheel_joint</right_joint>
            <wheel_separation>${2*base_radius}</wheel_separation>
            <wheel_radius>${wheel_radius}</wheel_radius>
            <odom_publish_frequency>10</odom_publish_frequency>
            <topic>cmd_vel</topic>
            <robotBaseFrame>base_link</robotBaseFrame>
        </plugin>
    </gazebo>
</robot>
```
2) Create the launch file
- Import the modules
```
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
```
- Create the _robot_description_ topic usin the _robot_state_publisher_
```
    xacro_path = 'urdf/diff_drive.urdf.xacro'
    robot_description = PathJoinSubstitution([
        get_package_share_directory('diff_drive_description'),	
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
- Spawn the robot
```
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'diff_drive',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.1',
                    '-r', '0',
                    '-p', '0',
                    '-Y', '0',
                    '-topic', '/robot_description'],
                 output='screen')

```
- Start Gazebo
```
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    
    
```
- Create the bridge
-- In this case, the cmd_vel is a topic tha travels from ROS2 to Gazebo, we can use the ] symbol
```
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                'cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )
    return LaunchDescription([        
        robot_state_publisher_node,
        spawn_node,
        ignition_gazebo_node,    
        bridge
    ])
```
3) Modify the CMakeLists.txt file
```
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
```
4) Compile the workspace and launch the simulation

       $ colcon build --symlink-install
       $ source install/setup.bash
       $ ros2 launch diff_drive_description diff_drive.launch.py
5) Use the rqt steering control to move the robot

        $ rqt 
        
## Example 6: Gazebo plugin
In this example we will learn how to develop new plugins for Gazebo. Developing a plugin that will be added to the Gazebo world configuration, or directly to the robot model (as made with the the differential drive control plugin), allow you to directly access to the simulated model, creating fastest control algorithms. We will discuss 3 examples. One standalone plugin, to understand the structure of a plugin. One integrating ROS2 and Gazebo and finally, one more complex plugin to control the differential drive robot. 

#### Basic plugin
This is not a ROS2 package, so we can create manually the directory structure. 

1) Let's create the hello_world plugin in the ROS2 worksapce. 

-- The main directory 
 
        $ mkdir hello_world

-- The main source file

        $ touch HelloWorld.cpp
        
-- The compilation file
        
        $ touch CMakeLists.txt

-- The world including the plugin

        $ touch hello_world_plugin.sdf

2) Fill the _HelloWorld.cpp_
- We'll use a string and the ignmsg command below for a brief example.
```
#include <string>
#include <gz/common/Console.hh>
```
- This header is required to register plugins, to be discovered to the Gazebo system
```
#include <gz/plugin/Register.hh>
```
- The System header is integrated to interface with the Gazebo system
```
#include <gz/sim/System.hh>
```
- It's good practice to use a custom namespace for your project.
```
namespace hello_world
{
```
- This is the main plugin's class. It must inherit from System and at least one other interface. Here we use `ISystemPostUpdate`, which is used to get results after physics runs. 
 ```
  class HelloWorld:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
  {
```
- Plugins inheriting ISystemPostUpdate must implement the PostUpdate callback. This is called at every simulation iteration after the physics updates the world. The _info variable provides information such as time, while the _ecm provides an interface to all entities and components in simulation
```
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
}
```
- This is required to register the plugin. Make sure the interfaces match what's in the header.
```
IGNITION_ADD_PLUGIN(
    hello_world::HelloWorld,
    gz::sim::System,
    hello_world::HelloWorld::ISystemPostUpdate)

using namespace hello_world;
```
- Here we implement the PostUpdate function, which is called at every iteration.
```
void HelloWorld::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
```
- This is a simple example of how to get information from UpdateInfo. Based on the status of the simulation (paused or not), we write a message on the terminal about the status. 
```
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";
  ignmsg << msg << std::endl;
}
```
3) Fill the CMakeLists.txt file, to directly compile the plugin
```
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
find_package(ignition-cmake2 REQUIRED)
project(Hello_world)
```
- We must find the compilation libraries and objects of the Gazebo toolkit. We can do this with the _ign_find_package_ keyword.
```
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
```
- A plugin is a library (it has not a main function). 
```
add_library(HelloWorld SHARED HelloWorld.cpp)
set_property(TARGET HelloWorld PROPERTY CXX_STANDARD 17)
target_link_libraries(HelloWorld
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
```
4) Create an SDF file to include the plugin. 
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
```
- Plugin filename: the one included in the CMakeLists.txt
- Plugin name: the class specified in the source file
```
    <plugin filename="HelloWorld" name="hello_world::HelloWorld">
    </plugin>
  </world>
</sdf>
```
5) Compile the plugin. This compilation is not based on colcon
```  
$ cd hello_world
$ mkdir build
$ cd build
$ cmake ..
$ make
$ cd ..
```  
6) Discover the plugin. This is made by setting the _GZ_SIM_SYSTEM_PLUGIN_PATH_ environment variable. This kinds of variable live only in the space where they are assigned. So we should export this variable in every terminal where the sdf world file is executed. Later we will see how to automatize this step
```
$ export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
``` 
7) Start the simulation. We must specify the sdf file, and the verbosity level. To print out data on terminal, the lowest verbosity level is 3.
``` 
$ ign gazebo -v 3 hello_world_plugin.sdf
``` 
Now you can check the terminal, how the message changes based on the start/stop status of the simulation. 

8) Avoid to create problems to the colcon compilation system. Now the _hello_world_ directory is in our ROS 2 software pool. This means that colcon will try to compile it. But this is not a ROS 2 package, so it will return an error. To avoid the compilation of this directory, create in its root an empty file called: COLCON_IGNORE

#### Gazebo Plugin <-> ROS 2 integration
Let's slightly modify the latter example to publish the status of the simulation not on the terminal but on a ROS 2 topic. In addition, we will use a launch file to start the simualtion.

1) Create a ROS 2 package 
``` 
$ ros2 pkg create hello_world_ros rclcpp std_msgs 
``` 
2) Create the needed directories 
``` 
$ cd hello_world_ros
$ mkdir launch
$ mkdir world
$ touch launch/hello_world_ros.launch.py
$ touch world/hello_world_ros_plugin.sdf
$ touch src/hello_world_ros.cpp
``` 

3) Fill the source code. This will be very similar to the previous one, let's highlight the difference with respect to the previous one.
-- Include the ROS 2 headers
``` 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
```
```
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace hello_world {
   class HelloWorldROS:
    public gz::sim::System,
```
-- In addition to the PostUpdate function, we will implement the Configure function. This is used to configure the environment of the plugin.
```
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate {
    
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    public: void Configure(const gz::sim::Entity &_id,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr) final;
```
-- Declare a pointer to the ROS 2 node. We will fill this object with the memory address of a ROS 2 node initialized object. Then we will create the publisher object.
```
    private: rclcpp::Node::SharedPtr _ros_node; 
    private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  };
}
IGNITION_ADD_PLUGIN(
    hello_world::HelloWorldROS,
    gz::sim::System,
    hello_world::HelloWorldROS::ISystemConfigure,
    hello_world::HelloWorldROS::ISystemPostUpdate)

using namespace hello_world;
```
-- Another function from the Gazebo tools is used the Configure function. This function is called when the plugin is loaded. Differently from the PostUpdate, called every time that the simulation iterates, the configure function is called just once.
```
void HelloWorldROS::Configure(const gz::sim::Entity &_entity,
    
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) {
```
-- In this function, we initialize the ROS 2 node without any argument (we don't have argc and argv)
```
    rclcpp::init(0, nullptr);
    _ros_node = rclcpp::Node::make_shared("hello_world_ros_plugin");
    _publisher = _ros_node->create_publisher<std_msgs::msg::String>("topic", 10);
}
```
```
void HelloWorldROS::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &/*_ecm*/) {
    std::string msg = "Hello, world! Simulation is ";
    if (!_info.paused)
        msg += "not ";
    msg += "paused.";
```
-- The ROS 2 string message is initialized, filled and published.
```
    auto message = std_msgs::msg::String();
    message.data = msg;
    _publisher->publish( message );
}
```
4) Fill the CMakeLists.txt. Starting from the one auto-generated from the ROS2 pkg create command, we must fuse the compilation instruction used from the Gazebo compilation. 

```
cmake_minimum_required(VERSION 3.8)
project(hello_world_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
```
- We must add the ignition-cmake2 package and the other components, we can copy and past these from the previous example
```
find_package(ignition-cmake2 REQUIRED)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
```
- We don't need an executable, but a library. Add this library, then, add to it the additional exteranl libraries
```
add_library(HelloWorldROS SHARED src/hello_world_ros.cpp)
target_link_libraries(HelloWorldROS 
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)
```
- And add the ROS 2 dependencies (thanks to the ament compilation system)
```
ament_target_dependencies(HelloWorldROS rclcpp std_msgs)
```
- Install the directories used in the launch process
```
install(DIRECTORY launch world DESTINATION share/${PROJECT_NAME})
```
```
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```
5) Fill the sdf world file. This is exactly the same of the previous example, we must just change the plugin.
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <plugin filename="HelloWorldROS" name="hello_world::HelloWorldROS">
    </plugin>
  </world>
</sdf>
```
6) Fill the launch file. Here we start the world but also set the environment. 
```
import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
```
-- We want to start with the sdf file defined above. We find the path of the shared package, setting the _hello_world_ros_plugin.sdf_. This path is used in the input arguments of the gazebo launch file. 
```
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
            'gz_args': f'-r -v 3 {sdf_file_path}'
        }.items()
    )

```
-- What misses...? The export of the plugin path. We can directly do it in the launch file. In this way, we can avoid to link our plugin to specific system configurations. Let's retrieve the build directory of the ROS 2 workspace and the _hello_world_ros_ subfolder. 
```
    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/hello_world_ros")    
```
-- The _SetEnvironmentVariable_ function can be used for this scope. Remember to add the element we are creating with this function in the action list of the LaunchDescription. 
```

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    
    ld.add_action( ign_resource_path)
    ld.add_action( gazebo_node)

    return ld
``` 
7) Compile the workspace and launch the node
``` 
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch hello_world_ros hello_world_ros.launch.py
$ ros2 topic echo /topic
``` 
__Be Careful__: if the system doesn't find the proper plugin, it will start the simulation anyway. No error are returned but of course, the system will not work as expected. 

### Publish on Gazebo topic
This last example still integrates ROS 2 and Gazebo, by controlling the differential drive robot by directly publishing on the Gazebo topic inside the plugin implementation. We will reproduce the behaviour of the ROS 2 bridge (it is just an example). The flow is:
-- Publish on a ROS 2 Topic called _cmd_vel_from_ros_
-- The _cmd_vel_from_ros_ is read from the Gazebo plugin
-- The data on the topic is managed and published on the Gazboe /cmd_vel topic

__Note__: we will add this plugin to a robot model. Let's add the urdf robot model of the differential drive robot here. 

1) Create the package
``` 
$ ros2 pkg create cmd_vel_plugin rclcpp std_msgs geometry_msgs 
``` 
2) Let's create the structure
``` 
$ cp -r diff_drive_description/urdf/ cmd_vel_plugin/
$ mkdir launch
$ touch src/pub_vel_cmd.cpp
``` 
3) Fill the source
-- As you can guess, the source is quite similar to the previous one, let's discuss the main differences
``` 
#include "rclcpp/rclcpp.hpp"
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
``` 
-- The Node header and the twist version of Gazebo are used to publish data
``` 
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/twist.hpp>
``` 
-- Let's define the namespace
``` 
namespace cmd_vel_plugin {
   class PubCmdVel:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate {
    
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    public: void Configure(const gz::sim::Entity &_id,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr) final;
``` 
-- Define the callback of the velocity message
``` 
    public: void cmd_vel_cb( const geometry_msgs::msg::Twist );
``` 
```
    private: rclcpp::Node::SharedPtr _ros_node; 
    private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
``` 
-- Define the subscriber 
```

    private: rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscriber;
    private: gz::transport::Node::Publisher _gz_cmdVelPub;
``` 
-- Define the Gazebo node object and the twist message (still gazebo side) 
```
    private: gz::transport::Node _gz_node;
    private: gz::msgs::Twist _cmdVelMsg;
  };
}


IGNITION_ADD_PLUGIN(
    cmd_vel_plugin::PubCmdVel,
    gz::sim::System,
    cmd_vel_plugin::PubCmdVel::ISystemConfigure,
    cmd_vel_plugin::PubCmdVel::ISystemPostUpdate)

using namespace cmd_vel_plugin;
``` 
-- In the callback of the velocity message, published in the ROS 2 network, we saturate the linear and angular velocities to 0.2 and 0.5.
```
void PubCmdVel::cmd_vel_cb( const geometry_msgs::msg::Twist t) {
    double vx = (t.linear.x < 0.2) ? t.linear.x : 0.2; 
    _cmdVelMsg.mutable_linear()->set_x (vx);
    double vz = (t.angular.z < 0.5) ? t.angular.z : 0.5;
    _cmdVelMsg.mutable_angular()->set_z(vz);
}
``` 
```
void PubCmdVel::Configure(const gz::sim::Entity &_entity, 
    const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) {

    rclcpp::init(0, nullptr);
    _ros_node = rclcpp::Node::make_shared("cmd_vel_plugin");
``` 
-- Create the subscriber to the ROS 2 message and the publisher to the /cmd_vel Gazebo topic
```
    _subscriber = _ros_node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_from_ros", 10, std::bind(&PubCmdVel::cmd_vel_cb, this, std::placeholders::_1));
    _gz_cmdVelPub = _gz_node.Advertise<gz::msgs::Twist>("/cmd_vel");
}
``` 
-- In the Update function, publish the message. We also have to call the spin function: the spin function allow the callbacks to work properly. We can use the sping_some on the ROS 2 node to avoid the blocking behaviour of the spin.
```
void PubCmdVel::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &/*_ecm*/) {    
    rclcpp::spin_some( _ros_node );
    _gz_cmdVelPub.Publish(_cmdVelMsg);
}
``` 
4) Modify the _CMakeLists.txt_. 
``` 
cmake_minimum_required(VERSION 3.8)
project(cmd_vel_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-cmake2 REQUIRED)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
``` 
-- We call the plugin CmdVelPlugin
``` 
add_library(CmdVelPlugin SHARED src/pub_cmd_vel.cpp)

target_link_libraries(CmdVelPlugin 
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)
ament_target_dependencies(CmdVelPlugin rclcpp std_msgs geometry_msgs)

install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
``` 
5) Add the plugin to the robot _xacro_
``` 
<gazebo>
        <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
            <left_joint>left_wheel_joint</left_joint>
            <right_joint>right_wheel_joint</right_joint>
            <wheel_separation>${2*base_radius}</wheel_separation>
            <wheel_radius>${wheel_radius}</wheel_radius>
            <odom_publish_frequency>10</odom_publish_frequency>
            <topic>cmd_vel</topic>
            <robotBaseFrame>base_link</robotBaseFrame>
        </plugin>
        <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
``` 
-- We call the plugin CmdVelPlugin, the class is _cmd_vel_plugin::PubCmdVel_
``` 
        <plugin filename="CmdVelPlugin" name="cmd_vel_plugin::PubCmdVel">
        </plugin>
    </gazebo>
``` 
6) Edit the launch file.
``` 
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    xacro_path = 'urdf/diff_drive.urdf.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('cmd_vel_plugin'),	
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
                    '-name', 'diff_drive',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.1',
                    '-r', '0',
                    '-p', '0',
                    '-Y', '0',
                    '-topic', '/robot_description'],
                 output='screen')

    
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 empty.sdf'
        }.items()
    )

``` 
-- The main difference except for the robot model, is the initialization of the environment variable.
``` 
    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/cmd_vel_plugin")    

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    
``` 
-- In the bridge, we don't need anymore the cmd_vel republish topic.
``` 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )


    return LaunchDescription([      
        ign_resource_path,  
        robot_state_publisher_node,
        spawn_node,
        gazebo_node,    
        bridge
    ])
``` 

7) Compile and test the plugin
``` 
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch cmd_vel_plugin cmd_vel_plugin.launch.py
```
-- Publish the cmd vel from rqt
```
$ rqt 
```

## Part 2: ros2_control and Gazebo

In this part of the tutorial, we will discuss of the ros2_control framework with two macro-examples. 
- Integration of the default controllers to control a robotic arm
- Development of a custom controller using the ros2_control framework

Before to start we need a robot to test the controllers. We will implement a 2 Degrees of Freedom (DOF) robot model for this. Let start to create the structure, without the controllers. 
1) Create the ROS 2 package.
```
$ ros2 pkg create pendulum_description --dependencies xacro
$ cd pendulum_description
$ mkdir urdf
$ mkdir launch
$ touch urdf/pendulum_robot.xacro
$ touch urdf/pendulum_no_controllers.launch.py
```
2) Write the robot model file _pendulum_robot.xacro_

```
<?xml version="1.0"?>

<robot name="pendulum_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="width" value="0.1" />  
  <xacro:property name="height_base" value="2" />   
  <xacro:property name="height_pole" value="1" />   
  <xacro:property name="axle_offset" value="0.05" /> 
  <xacro:property name="radius" value="0.03" /> 
  
   <link name="world"/>

  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${height_base/2}" rpy="0 0 0"/>
      <geometry>
	      <box size="${width} ${width} ${height_base}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 ${height_base/2}" rpy="0 0 0"/>
      <geometry>
	      <box size="${width} ${width} ${height_base}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${height_base/2}" rpy="0 0 0"/>
      <mass value="100"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>
  
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  
  <joint name="revolute_joint" type="continuous">
    <parent link="base_link"/>
    <child link="pole_link"/>
    <origin xyz="0.1 0 ${height_base - axle_offset}" rpy="0.1 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
  <link name="pole_link">
    <visual>
      <origin xyz="0 0 ${height_pole/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	      <cylinder length="${height_pole}" radius="${radius}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 ${height_pole/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	      <cylinder length="${height_pole}" radius="${radius}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${height_pole/2 - axle_offset}" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>
  <!-- ball Link -->
  <link name="ball_link">
    <visual>
      <origin xyz="0 0 ${height_pole}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 ${height_pole}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${height_pole}" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia
	  ixx="1.0" ixy="0.0" ixz="0.0"
	  iyy="1.0" iyz="0.0"
	  izz="1.0"/>
    </inertial>
  </link>
  <joint name="fixed_joint" type="fixed">
    <parent link="pole_link"/>
    <child link="ball_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```
3) Create the launch file (nothing of new here!)
```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import time
def generate_launch_description():

    ld = LaunchDescription()
    xacro_path = 'urdf/pendulum_robot.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('pendulum_description'),	
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
                    '-name', 'pendulum',
                    '-x', '1',
                    '-y', '1',
                    '-z', '0',
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
```
4) Modify the _CMakeLists.txt_
```
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```
4) Compile and start the simulation
```
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch pendulum_description pendulum_no_controllers.launch.py
```

As you can see, the robot falls under the effect of the dynamics factors, like the gravity. Why? 

#### Add position and state controllers to the robot
To allow the robot control we must add controllers. The main control interface is the position control, that allow us to specify the position of the joint. The controller will apply its best effort to reache the desired position. Another common interface is the velocity, where the setpoint is specified as a desired velocity command. 

To add the controllers follow these steps

1) Add the ros2_control tag to the xacro file. This tag is used to specify hardware interface and the joints involved in the control along with the interfaced used to control or read its state.  
Let's start from the previous xacro file, creating a new one called: _pendulum_robot_with_controllers.xacro_
```
<?xml version="1.0"?>

<robot name="pendulum_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="width" value="0.1" />  
  <xacro:property name="height_base" value="2" />   
  <xacro:property name="height_pole" value="1" />   
  <xacro:property name="axle_offset" value="0.05" /> 
  <xacro:property name="radius" value="0.03" /> 
  <link name="world"/>


  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${height_base/2}" rpy="0 0 0"/>
      <geometry>
	      <box size="${width} ${width} ${height_base}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 ${height_base/2}" rpy="0 0 0"/>
      <geometry>
	      <box size="${width} ${width} ${height_base}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${height_base/2}" rpy="0 0 0"/>
      <mass value="100"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <joint name="revolute_joint" type="continuous">
    <parent link="base_link"/>
    <child link="pole_link"/>
    <origin xyz="0.1 0 ${height_base - axle_offset}" rpy="0.1 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>


  <link name="pole_link">
    <visual>
      <origin xyz="0 0 ${height_pole/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	      <cylinder length="${height_pole}" radius="${radius}"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 ${height_pole/2 - axle_offset}" rpy="0 0 0"/>
      <geometry>
	      <cylinder length="${height_pole}" radius="${radius}"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 ${height_pole/2 - axle_offset}" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>



  <!-- ball Link -->
  <link name="ball_link">
    <visual>
      <origin xyz="0 0 ${height_pole}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 ${height_pole}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 ${height_pole}" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia
	  ixx="1.0" ixy="0.0" ixz="0.0"
	  iyy="1.0" iyz="0.0"
	  izz="1.0"/>
    </inertial>
  </link>

  <joint name="fixed_joint" type="fixed">
    <parent link="pole_link"/>
    <child link="ball_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```
- Add the ros2_control tag to the xacro file. 
```
<ros2_control name="IgnitionSystem" type="system">
```
- The hardware is the interface of the ignition to ROS2 control. We cannot chose a different controller
```
<hardware>
  <plugin>ign_ros2_control/IgnitionSystem</plugin>
</hardware>
```
- We have only 1 Joint the _revolute_joint_. Let's add to it two control interfaces: position and velocity. 
```
<joint name="revolute_joint">
  <command_interface name="position" />
  <command_interface name="velocity" />
```
- Similarly, we add a state inteface (to publish the well-known _/joint_state_ topic). In this case, we ask for the position and the velocity of the joint. In the position interface section, we can specify the initial position of the joint. In this case -1 rad. 
```
  <state_interface name="position">
    <param name="initial_value">-1.0</param>
  </state_interface>
  <state_interface name="velocity"/>
</joint>
</ros2_control>
```
2) Add the ros2_control plugin
```
<gazebo>
```
- The filename and the name of the plugin are these for Fortress, for Jazzy this plugin changes its name. 
```
    <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
```
- Each controller included here must be configured. To configure it, we use the classical yaml file. 
```    
        <parameters>$(find pendulum_description)/config/pendulum_controller.yaml</parameters>
    </plugin>
</gazebo>
```
3) Edit the yaml configuration file. Create a config directory in the _pendulum_description_ package. 
```
$ cd src/pendulum_description
$ mkdir config
$ touch config/pendulum_controller.yaml
```
- Here is the content. It is like a classical yaml used to store the parameters of a node. 
```
controller_manager:
  ros__parameters:
    update_rate: 100  
```
- A controller has a name. In this case we called it position_control. The name is used later to specify additional paremeters for the controller. The type of this controller is _forward_command_controller/ForwardCommandController_. This controller just forward the input data to the robotic joint.
```
    position_control:
      type: forward_command_controller/ForwardCommandController
```
- We do the same for the _velocity_control_. 
```
    velocity_control:
      type: forward_command_controller/ForwardCommandController
```
- We add also the joint_state_broadcaster, needed to publish the /joint_states. This controller doesn't needs for additional configurations, since it streams the data for all the joints of the robot.
```
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```
-  Now, it is possible to configure the controllers. For each of them we must define:
-- The joints involved in the control process
-- The input interface (position/velocity/effort)
```
position_control:
  ros__parameters:
    joints:
      - revolute_joint
    interface_name: position

velocity_control:
  ros__parameters:
    joints:
      - revolute_joint
    interface_name: velocity
 
```
4) Define the launch file. To spawn the new robot and load the controllers: _pendulum_controller.launch.py_.
```
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
```
- Load the new robot model, the one the controllers
```
    xacro_path = 'urdf/pendulum_robot_with_controllers.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('pendulum_description'),	
        xacro_path
    ])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'cart', '-allow_renaming', 'true'],
    )
```
- Each controller must be loaded and activated to have an effect on the joint. We can do this using the command line after loaded the robot model. At the same time we can do it in the launch file. 
-- To load the controller, we use the ExecuteProcess function in the launch file. Here we execute the following command:
        
        $ ros2 control load_controller --set-state active joint_state_broadcaster 
```
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_control'],
        output='screen'
    )

```
- We load, in an inactive state the velocity controllers, since we activated the position one 
```
    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'velocity_control'],
        output='screen'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
```
- The operations made on the controllers are executed at the end of the launch file using the _OnProcessExit_ function. This means that it is executed when the other nodes exist in the ROS 2 system.
```
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_position_controller],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_velocity_controller],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
    ])
```
5) Change the CMakeLists.txt
- Add the installation of the config directory
```
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```
6) Launch the simulation
- Compile and source the workspace
```
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch pendulum_description pendulum_controller.launch.py
```
- Check the active topics
```
$ ros2 topic list
    /joint_states
    /position_control/commands
    /velocity_control/commands
```
- Control the actuator, publishing a position command
```
$ ros2 topic pub /posion_control/commands std_msgs/msg/Float64MultiArray "{data: [3]}"
```

7) Interact with the controller manager

```
$ ros2 control list_controllers
        list_controllers -> Output the list of loaded controllers, their type and status
```
```
$ ros2 control list_hardware_interfaces
        list_hardware_interfaces -> Output the list of available command and state interfaces

```

8) Change the active controller

```
$ ros2 control switch_controllers --activate velocity_control --deactivate position_control
$ ros2 control list_controllers
$ ros2 topic pub /veloty_control/commands std_msgs/msg/Float64MultiArray "{data: [2]}"
```

#### Develop a custom controller
Sometimes it is useful to create custom controller to perform specific actions. Define a specific controller is useful when we want to install routines directly inside the controller. This improve the performance of the controller. Let's create a controller that controls the angular commands of the _revolute_joint_ following a sinusoidal motion.

1) Create the controller package

```
$ ros2 pkg create sine_ctrl --dependencies rclcpp std_msgs controller_interface pluginlib
$ cd sine_ctrl
$ touch sine_controller.xml
$ src/sine_ctrl.cpp
```

2) Create the controller source
- At start, we include the header files to use the ROS2 function, access to the controller interface and handle Flota32MultiArray data
```
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

- The controller must be implemented inside a namespace. Inside the namespace, we must define the controller class that inherits from the base class controller_interface::ControllerInterface:

namespace sine_controller {
  class SineController : public controller_interface::ControllerInterface
  {
  
- Among the different variables, we have to provide access to the joint state interface, to the current state of an actuator (position, velocity, effort). The ros2_control framework implements this with the LoanedStateInterface, which provides temporary, controlled access to state data for real-time operations, ensuring safe and synchronized interactions within a controller. It helps manage access to state data without causing conflicts or inconsistencies in the system:  
  
  
  private:  
    rclcpp::Duration _dt;

- To use LoanedStateInterface we define a type alias for a 2D vector of std::reference_wrapper<T>. The variable
joint_state_interfaces_ uses this type to manage references to hardware_interface::LoanedStateInterface objects:
    
    
    template<typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
    
- Finally, a vector of string vectors is declared to define the interface name. We have to consider a new name for each joint and interface, for example: joint_name/position, joint_name/velocity, joint_name_2/position, and so on:    
    
    std::vector<std::vector<std::string>> state_interface_names_;
    
    int _j_size = 1;
    float _initial_joint_position;
    double _desired_joint_positions;
    float _amplitude;
    float _frequency;
    double t_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _sine_param_sub;

  public:
    SineController() : controller_interface::ControllerInterface(), _dt(0, 0) {}


- The callback uses the received data to fill the amplitude, and the frequency defined in as class members. First, we check that the format of the data is correct. We need 4 numbers to set the parameters correctly



    void sine_param_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)  {
    
      if( msg->data.size() != 2 ) {
        cout << "Wrong number of sine parameters" << endl;
        return;
      }
      
      _amplitude = msg->data[0];
      _frequency = msg->data[1];
      
    }


- We are now ready to define the state and the command interfaces. These functions are called automatically by the controller manager:
  
    
    // setta dove andare a prendere lo stato (virtuale) - hw interface
    controller_interface::InterfaceConfiguration state_interface_configuration() const {
    
- Here we can hardcode the name of the state interfaces. The syntax is: joint_name/interface. Since in the configuration of the joints we share the position and the velocity as joint states, and we have two joints, we will add
4 elements to the state_interfaces_config_names vector:
    
    
      std::vector<std::string> state_interfaces_config_names;
      state_interfaces_config_names.push_back("revolute_joint/position");
      //state_interfaces_config_names.push_back("revolute_joint/velocity");
  
  
- To effectively set the interfaces, we have to return the interface list. In this case, we specify them using the INDIVIDUAL property, that is specify a detailed list of required interfaces, formatted as <joint_name>/<interface_type>:
  
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
    }
    
The same thing is done with the command interfaces. Of course, in this case we just specify the position, since we want to control the position of the joints
    
    controller_interface::InterfaceConfiguration command_interface_configuration() const {
      std::vector<std::string> command_interfaces_config_names;
      command_interfaces_config_names.push_back("revolute_joint/position");
      
      
And, also, in this case, we turn the list of the INDIVIDUAL interfaces participating in the motor control:
      
      
     
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces_config_names};
    }

- In the on_configure function, we can do all the operations needed before starting the controllers. Here, we initialize the subscriber of the sinusoidal parameters

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      
      _sine_param_sub =   get_node()->create_subscription<std_msgs::msg::Float32MultiArray>("/sine_param", 10, 
            std::bind(&SineController::sine_param_cb, this, std::placeholders::_1));


As for the _dt variable, it is the step time of the update function. The update_rate_ param is used to retrieve the rate of the controller from the ROS2 parameter server. As seen, we specified the parameters in the .yaml file to configure the controllers

      _dt = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / update_rate_));

      _amplitude = 0.0;
      _frequency = 0.0;

- Then, we resize the command to the interface vectors. 

      command_interfaces_.reserve     (_j_size); 
      state_interfaces_.reserve       (_j_size);
      joint_state_interfaces_.resize  (_j_size);
      state_interface_names_.resize   (_j_size);
      
    
- We also initialize a list of the interfaces, we will use it later.    
    
      std::vector<std::string> joint_name = {"revolute_joint"};
      for(int i = 0; i < _j_size; i++) {
        state_interface_names_[i].resize(1);
        state_interface_names_[i][0] = joint_name[i] + "/position";
      }


- This function must return the eventual SUCCESS or FAILURE of the initialization. Imagine that in this function you need to read some parameters. If the params don't exist or are not correctly specified, you could get a FAILURE in return; though, it’s SUCCESS in our case
  
      RCLCPP_INFO(get_node()->get_logger(), "configure successful");
      return controller_interface::CallbackReturn::SUCCESS;
    }

- Another function to implement is the on_activate function. This function is called when the controller passes from an inactive to active state:

    // quando il controllore passa in attivo
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
    
- We call the get_ordered_interfaces function from the controller_interface namespace that is used to retrieve and organize a
list of interfaces, specifically state_interfaces:    
    
    
      for(int i = 0; i < 1; i++) 
        controller_interface::get_ordered_interfaces( state_interfaces_, state_interface_names_[i], std::string(""),joint_state_interfaces_[i]);
      
. Then, we retrieve the joint position to save the initial value of the motor control:      
      
      
      _initial_joint_position = joint_state_interfaces_[0][0].get().get_value();
      RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
      t_ = 0.0;



      return controller_interface::CallbackReturn::SUCCESS;
    }

- We don’t do any operation in the on_init and deactivate functions
  
    // on init viene chiamato quando c' il load del controllore
    controller_interface::CallbackReturn on_init() {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
      return controller_interface::CallbackReturn::SUCCESS;
    }
    
- Finally, we can write the update function. Here, we must calculate the desired values of the joints that follow the sinusoidal profile:
    
    
    controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    
- First, we increment the local time variable that is used to proceed with the sinusoidal signal. Then, for each joint, we calculate the desired position with the classical sinusoidal formula: x(t) = Amplitude*sin(2*pi*Frequency*time)
    
    
        t_ += _dt.seconds();
        _desired_joint_positions = _initial_joint_position + (_amplitude * std::sin(2 * M_PI * _frequency * t_));
        
- Finally, we can set the desired position value using the command interface.        
        

        command_interfaces_[0].set_value(_desired_joint_positions);
        return controller_interface::return_type::OK;
    }
  };
} // namespace sine_controller

- We are now ready to install the plugin. At this stage, we use the PLUGINLIB_EXPORT_CLASS macro. This macro is defined into the pluginlib/class_list_macros.hpp header file. For this reason, we need to include this header file in our source code. To successfully export the controller, we must specify as first parameter the class containing the controller
implementation, like <controller_name_namespace>::<ControllerName>, and as the second parameter the base class of the controller, that is controller_interface::ControllerInterface.

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sine_controller::SineController, controller_interface::ControllerInterface)
```

3) Add the xml file
```
<library path="sine_controller">
  <class name="sine_controller/SineController" type="sine_controller::SineController" base_class_type="controller_interface::ControllerInterface">
  <description>
    The joint controller commands a group of joints in a given interface
  </description>
  </class>
</library>
```
4) Edit the _CMakeLists.txt_
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)


add_library(sine_controller SHARED src/sine_ctrl.cpp)
pluginlib_export_plugin_description_file(controller_interface sine_controller.xml)


ament_target_dependencies(sine_controller
  rclcpp
  controller_interface
  pluginlib
)
install(
  TARGETS sine_controller
  LIBRARY DESTINATION lib
)
```


5) Add the controller to the robot
- Edit the dependencies (via the _CMakeListst.txt_)
```
find_package(sine_ctrl REQUIRED)
```
- Launch the controller in the launch file
```
load_sine_controller = ExecuteProcess ( 
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'sine_controller'],
        output='screen'
)
return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
       
        node_robot_state_publisher,
        gz_spawn_entity,
        load_joint_state_broadcaster,
        load_position_controller,
        load_velocity_controller,
        load_sine_controller
    ])
```
6) Start the sineusoidal controller
```
$ ros2 control switch_controllers --activate sine_controller --deactivate position_control
$ ros2 topic pub /sine_param std_msgs/msg/Float32MultiArray "{data: [1.3, 0.3]}"
```





















