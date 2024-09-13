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


# ros2_control and Gazebo



