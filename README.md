# ROSCon_Es 2024 tutorial: Modern Gazebo y integraciòn con ros2_control

Este repository contiene el código fuente para seguir el tutorial de ROSConES 2024 titulado: _Tutorial de Gazebo e integración con ros2_control_ realizado por Jonathan Cacace de la __Unidad de Robótica de Eurecat__  
__Horario__: 19 de septiembre de 2024, 9:30-11:30

## Empezando
### Requisitos
El tutorial está diseñado para ROS2-Humble (LTS). Puedes instalarlo en tu sistema (Ubuntu 22.04) o en un contenedor Docker. El repositorio contiene los archivos necesarios para crear la imagen Docker e instanciar el contenedor.

### Descargar el repositorio
Puedes descargar este repositorio en cualquier lugar en tu sistema. Se asume que lo clonarás en tu carpeta personal (home)

		$ cd ~ 
		$ git clone https://github.com/jocacace/ROSConES.git

### Build y ejecutar la imagen
	
		$ cd ROSConES/Docker 		
		$ docker build -t ros2roscon .		
		$ docker compose up # You will lose the control of this terminal tab

Para adjuntar un nuevo terminal a este contenedor Docker, usa el comando _docker exec_	

		$ docker exec -it ros2roscon_container bash

Este último comando abre el acceso al contenedor Docker en el nivel superior de la carpeta Docker. Este será tu espacio de trabajo de ROS 2 (la ros workspace). Por ejemplo, aquí puedes compilar el espacio de trabajo:

		$ colcon build --symlink-install
		$ source /opt/ros/humble/setup.bash
		
Ahora puedes comenzar a compilar y ejecutar los ejemplos!

### Indice

PART 1:
- [Introducciòn a Gazebo](#introducciòn-a-gazebo)
- [Example 1: Key Publisher in Gazebo](#key-publisher-from-gazebo)
- [Example 2: Spawn un objecte en la simulación](#spawn-an-object-in-the-scene)
- [Example 3: Agregar un sensor en gazebo](#sensores)
- [Example 4: ROS2 Bridge](#ros2-bridge)
- [Example 5: Differential drive robot](#differential-drive-robot)
- [Example 6: Gazebo plugin](#gazebo-plugin)

PART 2:
- [ros2_control and Gazebo](#ros2_control-and-gazebo)
- [Develop a custom controller](#develop-a-custom-controller)



## Parte 1: Empezamos con modern Gazebo (Fortress)

## Instalación de Gazebo

Para instalar Gazebo, si no usas Docker, ejecuta los siguientes comandos:

```
$ sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg 

$ sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null 

$ sudo apt-get update 

$ sudo apt-get install ignition-fortress ros-humble-ros-ign-bridge ros-humble-ros-gz ros-humble-controller-manager ros-humble-ros2-control ros-humble-ros2-controllers 
ros-humble-ign-ros2-control -y
```

## Introducciòn a Gazebo 
Puedes interactuar con la simulación de Gazebo utilizando el terminal de Linux con el comando: ign.

	The 'ign' command provides a command line interface to the ignition tools.
		ign <command> [options]

Para comenzar la simulacion:
		
		$ ign gazebo

Esto abrirá una página de bienvenida donde puedes seleccionar una escena lista o una escena vacía. Esta escena es bastante inutilizable, ya que también le falta el plano del suelo (Si agregas un objeto a la escena, caerá.).
Para abrir directamente una escena vacía, usa este comando:


		$ ign gazebo empty.sdf

Este es un mejor punto de partida. En esta ventana, además del escenario principal, hay elementos de la interfaz gráfica:  
- **World**: todos los elementos  
- **Entity Tree**: los objetos

## Gazebo topics
Internamente, Gazebo comparte información utilizando topics, exactamente como ROS 2. Usaremos esta información para la integraciòn de ROS 2 y Gazebo luego. Algunos comandos son útiles para entender los datos activos en el sistema:


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

Por ejemplo, para listar los topicos activos:
		
    $ ign topic -l

## Example 1: 
## Key Publisher from Gazebo
- Abre Gazebo con una escena vacía

		$ ign gazebo empty.sdf

- Añade el plugin Key Publisher: 
  - haz clic en los 3 puntos verticales -> busca y selecciona el plugin Key Publisher

- Inicia la simulación con el botón de reproducción

- En un terminal, verifica los temas activos actuales:

		$ ign  topic -l
- Escucha el contenido del topico: _/keyboard/keypress_

		$ ign topic -e -t /keyboard/keypress
- Ahora, usa el teclado en la escena de Gazebo y verifica el output en el terminal.

Ahora tenemos los elementos básicos para usar Gazebo con ROS2.

## Example 2: 
## Spawn an object in the scene
Gazebo es un software independiente y es agnóstico a ROS 2. Soporta SDF como modelo de objeto: un robot o un objeto estático. SDF significa Simulation Description Format. Puedes encontrar diferentes simulaciones con robots complejos que utilisan directamente SDF. Sin embargo, para una integración más profunda con ROS, preferimos el uso del formato de archivo URDF.el espacio de trabajo

Vamos a crear un nuevo paquete que vas a contar  el modelo de un objeto simple y su launch file:

	$ ros2 pkg create spawn_simple_object --dependencies xacro
	$ cd spawn_simple_object
	$ mkdir launch
	$ mkdir urdf
	$ cd urdf && touch cube.urdf.xacro

El modelo del robot es el siguiente. Es solo un link con la forma de un cubo.
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

Utilizamos un launch file para añadir el robot a la simulación. Vamos a escribir esto file. 

		$ cd spawn_simple_object/launch
		$ touch spawn_model.launch.py


- Empezamos importando los módulos relevantes
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
- Recupera el archivo del modelo del robot (el xacro)
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
- Spwan el robot a partir dal topic _robot_description_
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
- Añadamos el módulo para iniciar la simulación:
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
- Instalamos los directorios del paquete con el _CMakeLists.txt_
```
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
```
Ahora puede compilar y cargar la  ROS workspace antes de iniciar la simulación

    $ colcon build --symlink-install
    $ source install/setup.bash
    $ ros2 launch spawn_simple_object spawn_model.launch.py 

## Example 3: 
## Sensores
Complicaremos nuestro modelo añadiendo algun sensores adicionales. En particular, una cámara estándar, una cámara de profundidad y un sensor lidar. Los sensores se añaden al mismo cube que simulamos antes. 
Un parámetro de input a las launch file define qué sensor usar en la simulación.
Este paquete utiliza los paquetes _realsense2-camera_ y _realsense2-description_ para simular el sensor de profundidad.

    $ sudo apt-get install ros-humble-realsense2-camera
    $ sudo apt-get install ros-humble-realsense2-description
    
Vamos a crear el nuevo paquete _gazebo_sensors_.

    $ ros2 pkg create gazebo_sensors --dependencies xacro realsense2_description
    
### Define el modelo (xacro) 

Editamos el nuevo modelo

	$ cd gazebo_sensors
	$ mkdir urdf
	$ touch urdf/cube_with_sensors.urdf.xacro

Aqui tenemos algún parameters para decidir que sensore activar.
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
- Vamos a definir el base_link y el sensor link, conectado con un fixed joint. Los sensores saran conectate con el sensor_link
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
- Si el parámetro _use_camera_ està True, añade el sensor de la cámara
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
- Si el paràmetro _use_depth_ camera està true, añade el depth sensor. 
```
<xacro:if value="${use_depth}">         
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
- Si el paràmetro _use_lidar_ esta True, añade el lidar.
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
- Necesitamos añadir el plugin para los sensores tambien. En esta nueva versión de Gazebo, tenemos un plugin para todos los sensores. Recuerda en el Gazebo clásico, cada sensor tiene su propio plugin.

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
</robot>
```
#### Creamos el launch file

	$ mkdir launch
	$ touch launch/cube_with_sensors.launch.py



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
- Añadamos el modelo de descripción del robot, inicializando con los parámetros. En el primer caso, _use_camera_ esta True.
```
def generate_launch_description():
    ld = LaunchDescription()
    xacro_path = 'urdf/cube_with_sensors.urdf.xacro'
 
```
- A partir del xacro, podemos instanciarlo con los parámetros.
```
    robot_description = xacro.process_file( Path(os.path.join( get_package_share_directory('gazebo_sensors'), xacro_path ) ), mappings={'use_camera': "True", 'use_depth': "False", 'use_lidar': "False"})
```
- Define el nodo de publicación del robot state publisher. Este nodo publicó el topic _robot_description_.

```
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
```
- Hemos cargado la descripción del robot del xacro, esta ya traducida en URDF con los parámetros

```
            'robot_description':robot_description.toxml()
        }]
    )
```
- Vamos a spawnar el robot. Se utiliza el tema _robot_description_

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
- El nodo para iniciar la simulacion
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
### Vamos a crear el _CMakeLists.txt_
```
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
```
### Ahora puedes compilar la ROS workspace y lanzar la simulación.

      $ colcon build --symlink-install
      $ source install/setup.bash
      $ ros2 launch gazebo_sensors cube_with_sensors.launch.py

Para cada sensor hay un Gazebo plugin para comprobar el output de los sensores:
- Image display (image and depth)
- Visualize lidar

Check the topic published on ignition
    
        $ ign topic -l

### Example 4: 
### ROS2 Bridge

Los datos en Gazebo no son útiles si no podemos utilizarlos mediante ROS. Para transferir los datos de Gazebo a ROS 2 debemos utilizar el _ros_gz_bridge_.

El bridge creará un puente que permite el intercambio de mensajes entre ROS e Gazebo. Su compatibilidad está limitada a determinados tipos de mensajes. No todos son compatibles ahora. 
El bridge se puede utilizar de dos formas:
- Command line ROS program
- Launch file

La sintaxis es 

      $ ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@GZ_MSG

El comando ros2 run _ros_gz_bridge_ _parameter_bridge_ simplemente ejecuta el código parameter_bridge del paquete ros_gz_bridge. 
Luego, especificamos el nuestro topic nombre /TOPIC utilizado para transmitir los mensajes. El primer símbolo @ delimita el nombre del tema de los tipos de mensajes. Después del primer símbolo @ se encuentra el tipo de mensaje de ROS.

El tipo de mensaje ROS va seguido de un símbolo @, [ o ] donde:

    @ is a bidirectional bridge.
    [ is a bridge from Gazebo to ROS.
    ] is a bridge from ROS to Ignition.

#### A simple case: Publish key strokes to ROS
- Después haber iniciado Gazebo anda el plugin Key Publisher: 

  - haga clic en los 3 puntos verticales -> busque y haga clic en el plugin Key Publisher.

- Obtienes el tipo de mensaje Gazebo con este comando
    
        $ ign topic -i -t /keyboard/keypress
            Publishers [Address, Message Type]:
            tcp://172.18.0.1:37005, ignition.msgs.Int32

- Inicia el bridge
    
        $ ros2 run ros_gz_bridge parameter_bridge /keyboard/keyprs@std_msgs/msg/Int32[ignition.msgs.Int32

- Escucha el messaje en ROS

        $ ros2 topic echo /keyboard/keyprs
        
#### Bridge the Sensor data
Vamos a crear un launch file que inicializa los tres bridges para los tre sesores.

1) Inicie la simulación y obtenga la información sobre los topics que queremos abordar en ROS 2

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

2) Vamos a crear un launch file para iniciar el bridge: _gazebo_bridge.launch.py_ 

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
- Tambien, podemos mapear el topic para cambiar el nombre de los datos publicados por Gazebo, para adaptarlo al resto del sistema ROS2.
```
        remappings=[
            ("/lidar", "/cube/lidar"),
        ],
        output='screen'
    )
```
- Necesitamos publicar manualmente la transformación de el output del sensor al base frame de la cámara de profundidad y de el lidar. En esta manera podems visualizar todo los datos en el robot base frame
```
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
```
3) Ahora podemos usar rqt para visualizar los datos. En el docker ya esta todo instalado. Sin embargo, tu puede instalar todo los que necesita con este comandos
- rqt: 

        $ sudo apt-get install ros-humble-rqt-*
        $ ros2 run rqt-image-view rqt-image-view
- rviz2

  - Con Rviz2 es posible visualizar el lidar y los PointClouds simplemented añadendo el plugin adecuado utilisando el plugin menu

## Example 5: 
## Differential drive robot

Comencemos a implementar nuestro primer robot móvil. Esto también permitirá empezar con los robot controladores.

- Vamos a crear un robot con 2 ruedas pasivas y 2 ruedas activas.

      $ ros2 pkg create diff_drive_description --dependencies xacro
      $ mkdir diff_drive_description/urdf
      $ mkdir diff_drive_description/launch

1) Crea dos archivos: uno para contener los macros y uno por el xacro principal

- Vamos a definir los macros para crear el modelo del robot, la inercia del cilindro y otros.
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

```
    <xacro:macro name="passive_wheel_joint" params="name parent child *origin">
      <joint name="${name}" type="fixed" >
        <parent link="${parent}" />
        <child link="${child}" />
        <xacro:insert_block name="origin" />
      </joint>
    </xacro:macro>
```

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
-- Ahora podemos crear el archivo _xacro_. Primero incluya el archivo de macro
```
<?xml version="1.0"?>

<robot name="diff_robot" xmlns:xacro="http://ros.org/wiki/xacro">
<xacro:include filename="$(find diff_drive_description)/urdf/diff_drive_macro.xacro" /> 
```
-- Definimos alcun parametros
```
    <xacro:property name="base_radius" value="0.15" /> 
    <xacro:property name="passive_wheel_height" value="0.04" /> 
    <xacro:property name="passive_wheel_radius" value="0.025" /> 
    <xacro:property name="passive_wheel_mass" value="0.5" /> 
    <xacro:property name="wheel_radius" value="0.039" /> 
    <xacro:property name="wheel_height" value="0.02" />
    <xacro:property name="wheel_mass" value="2.5" /> 
```
--Y la structura del robot
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

```
	<xacro:wheel side="right" parent="base_link" translateX="0" translateY="-${base_radius}" />
	<xacro:wheel side="left" parent="base_link" translateX="0" translateY="${base_radius}" />
```

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
  - Este robot tiene dos sensores
    - __Sensor plugin__: con esto transmitimos el lidar sensor  
    - __Differential drive plugin__: con esto controlamos la velocidad de la base, traduciendo esta velocidad en la velocidad de las ruedas.

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
2) Vamos a crear un launch file
- Importamos los modules
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
- Creamos los _robot_description_ topic usand el paquete _robot_state_publisher_
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
- Spawn el robot
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
- Iniciamos Gazebo
```
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    
    
```
- Creamos el bridge
  - En esto caso, el topic cmd_vel esta suscrito from ROS2 to Gazebo, podemos usar el ] symbol
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
3) Modificar el file _CMakeLists.txt_
```
install(DIRECTORY launch urdf DESTINATION share/${PROJECT_NAME})
```
4) Compilamos la workspace y lainziamos la simulación


       $ colcon build --symlink-install
       $ source install/setup.bash
       $ ros2 launch diff_drive_description diff_drive.launch.py
5) Vamos a usar el rqt steering control to actuate el robot

        $ rqt 
        
## Example 6: 
## Gazebo plugin
En este ejemplo vamos a desarrollar plugins personalizados para Gazebo. Desarrollar un plugin que se añadirà a la configuración del mundo de Gazebo o directamente al modelo del robot (como se hizo con el differential drive plugin).

Un gazebo plugin permitirá de acceder directamente al modelo simulado y crear algoritmos de control más rápidos y performante. 
Vamos a veer 3 ejemplos. 
- Un plugin independiente para comprender la estructura de un plugin. 
- Uno que integra ROS 2 y Gazebo 
- Un plugin más complejo para controlar el robot con transmisión diferencial.

### Basic plugin
Este no es un paquete ROS, entonces, tenemos que crear manualmente la estructura del paquete.

1) Creemos el plugin _hello_world_ en el ROS workspace. 

 
        $ mkdir hello_world
        $ touch HelloWorld.cpp
        $ touch CMakeLists.txt
        $ touch hello_world_plugin.sdf

2) Editamos el _HelloWorld.cpp_

- En esto ejemplo vamos a escribir un mensaje su la console de Gazebo.

```
#include <string>
#include <gz/common/Console.hh>
```
- Vamos a incluir un header para registrar el plugin en el grupo del plugin utilizable por Gazebzo.
```
#include <gz/plugin/Register.hh>
```
- El header por conectar el código con el sistema Gazebo
```
#include <gz/sim/System.hh>
```
- Vamos a definir un namespace
```
namespace hello_world
{
```
- Esta es la clase principal del plugin. Debe heredar de System y al menos otra interfaz. Aquí usamos `ISystemPostUpdate`, que se utiliza para obtener resultados después la iteracion del simulator.
 ```
  class HelloWorld:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
  {
```
- Los plugins que heredan ISystemPostUpdate deben implementar la llamada PostUpdate. Esta se llama en cada iteración de simulación después de que la física actualiza el mundo. 
- La variable _info proporciona información como la hora, mientras que _ecm proporciona una interfaz para todas las entidades y componentes de la simulación.
```
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
}
```
- Esto es necesario para registrar el plugin.
```
IGNITION_ADD_PLUGIN(
    hello_world::HelloWorld,
    gz::sim::System,
    hello_world::HelloWorld::ISystemPostUpdate)

using namespace hello_world;
```
- Vamos a implementar el PostUpdate function, que se llama en cada iteración.
```
void HelloWorld::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
```
- Este es un ejemplo sencillo de cómo obtener información de UpdateInfo. En función del estado de la simulación (en pausa o no), escribimos un mensaje en la terminal sobre el estado.
```
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";
  ignmsg << msg << std::endl;
}
```
3) Vamos a editar el _CMakeLists.txt_ para compilar el plugin
```
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
find_package(ignition-cmake2 REQUIRED)
project(Hello_world)
```
-  Debemos encontrar las librerías de compilación y los objetos del kit de compilacion de  Gazebo. Podemos hacerlo con la palabra clave _ign_find_package_.
```
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
```
- Un plugin es una shared library (falta la main function). 
```
add_library(HelloWorld SHARED HelloWorld.cpp)
set_property(TARGET HelloWorld PROPERTY CXX_STANDARD 17)
target_link_libraries(HelloWorld
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
```
4) Vamos a crear el SDF file para incluir el plugin. 
```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
```
- Plugin filename: Eso utilisado en el CMakeLists.txt para compilar la shared library
- Plugin name: Eso utiliado en le source file 
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
6) Descubrir el plugin. Esto se hace configurando la environment variable  _GZ_SIM_SYSTEM_PLUGIN_PATH_. Recuerda: este tipo de variables solo viven en el espacio donde están asignadas. Por lo tanto, deberíamos exportar esta variable en cada terminal donde se ejecute el archivo sdf world. Más adelante veremos cómo automatizar este paso.

```
$ export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
``` 
7) Iniciamos la simulación. Debemos especificar el archivo sdf y el nivel de verbosidad. Para imprimir los datos en la terminal, el nivel de verbosidad más bajo es 3.
``` 
$ ign gazebo -v 3 hello_world_plugin.sdf
``` 
- Ahora puedes comprobar en la terminal cómo cambia el mensaje en función del estado de inicio o detención de la simulación.

8) Evite usar colcon con esto plugin

- Ahora el directorio _hello_world_ está en nuestro pool de software de ROS 2. Esto significa que colcon intentará compilarlo. Pero no es un paquete de ROS 2, por lo que devolverá un error. Para evitar la compilación de este directorio, vamos a crear un archivo llamado COLCON_IGNORE en esto directorio

### Gazebo Plugin <-> ROS 2 integration
Modifiquemos ligeramente el último ejemplo para publicar el estado de la simulación no en el terminal sino en un tema de ROS 2. Además, utilizaremos un archivo de inicio para iniciar la simulación.

1) Vamos a crear un nuevo paquete 
``` 
$ ros2 pkg create hello_world_ros rclcpp std_msgs 
``` 

``` 
$ cd hello_world_ros
$ mkdir launch
$ mkdir world
$ touch launch/hello_world_ros.launch.py
$ touch world/hello_world_ros_plugin.sdf
$ touch src/hello_world_ros.cpp
``` 

3) Vamos a rellenar el código, este será muy similar al primero ejemplo. Vamos a resaltar la diferencia entre los dos.

  - ROS2 headers
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
  - Vamso a añadir el methodo: Configure para initializar el plugin.
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
-- Declaramos un puntero al nodo ROS 2. Initializiamo esto ne la configuracion. 

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

```
void HelloWorldROS::Configure(const gz::sim::Entity &_entity,
    
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) {
```
  - En esta función, inicializamos el nodo ROS 2 sin ningún argumento (no tenemos argc ni argv)
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
  - Al final, el mensaje de  ROS 2 se inicializa, se completa y se publica.

```
    auto message = std_msgs::msg::String();
    message.data = msg;
    _publisher->publish( message );
}
```
4) Complete el archivo CMakeLists.txt. A partir del archivo generado automáticamente, debemos fusionar la instrucción de compilación utilizada en la compilación de Gazebo.

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
- Debemos agregar el paquete ignition-cmake2. Podemos copiarlos y pegarlos del ejemplo anterior
```
find_package(ignition-cmake2 REQUIRED)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
```
- No necesitamos un ejecutable. Diferentemente, creamos una library y luego, añadimos les dependencias.

```
add_library(HelloWorldROS SHARED src/hello_world_ros.cpp)
target_link_libraries(HelloWorldROS 
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)
```
```
ament_target_dependencies(HelloWorldROS rclcpp std_msgs)
```
- Vamos a instalar los directorios utilizados en el proceso de lanzamiento del plugin
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
5) Vamos a Rellenar el archivo de el world. Es exactamente lo mismo que el ejemplo anterior, solo debemos cambiar el complementplugino.

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <plugin filename="HelloWorldROS" name="hello_world::HelloWorldROS">
    </plugin>
  </world>
</sdf>
```
6) Editamos el launch file para iniciar el mundo e inicializar las variables de ambiente.
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
  - ¿Qué falta? La exportación de la ruta del plugin. Esto podemos hacerlo directamente en el launch file. De esta forma, podemos evitar vincular nuestro plugin a configuraciones específicas del sistema. Recuperemos el directorio de compilación del workspace de ROS 2 y la subcarpeta _hello_world_ros_.

```
    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/hello_world_ros")    
```
```
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    
    ld.add_action( ign_resource_path)
    ld.add_action( gazebo_node)

    return ld
``` 
7) Compilamos y empeziamos el plugin
``` 
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch hello_world_ros hello_world_ros.launch.py
$ ros2 topic echo /topic
``` 
__Be Careful__: Si el sistema no encuentra el plugin, iniciará la simulación de todos modos. No se devuelve ningún error, pero, por supuesto, el sistema no funcionará como se espera.

### Publish on Gazebo topic
Este último ejemplo aún integra ROS 2 y Gazebo para controlar el robot differencial mediante la publicación directa en el tema Gazebo dentro de la implementación del complemento. Reproduciremos el comportamiento del bridge ROS 2 (es solo un ejemplo). 

El flujo es:
  - Publicar en un topic ROS 2 llamado _cmd_vel_from_ros_
  - El _cmd_vel_from_ros_ se lee adentro el plugin Gazebo
  - Los datos del topic se modifica un pochito y se publican en el topic Gazboe /cmd_vel


__Note__: Agregaremos esto plugin al model del robot en el urdf

1) Vamos a crear el paquete
``` 
$ ros2 pkg create cmd_vel_plugin rclcpp std_msgs geometry_msgs 
``` 
``` 
$ cp -r diff_drive_description/urdf/ cmd_vel_plugin/
$ mkdir launch
$ touch src/pub_vel_cmd.cpp
``` 
3) Vamos a editar el codigo

- El codigo es bastantemente similar al codigo del ejemplo anterior
``` 
#include "rclcpp/rclcpp.hpp"
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
``` 
``` 
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/twist.hpp>
``` 
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
``` 
    public: void cmd_vel_cb( const geometry_msgs::msg::Twist );
``` 
```
    private: rclcpp::Node::SharedPtr _ros_node; 
    private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
``` 
```

    private: rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscriber;
    private: gz::transport::Node::Publisher _gz_cmdVelPub;
``` 
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
- En la callback de el mensaje en la red ROS 2 saturamos las velocidades lineales y angulares a 0.2 y 0.5.

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
```
    _subscriber = _ros_node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_from_ros", 10, std::bind(&PubCmdVel::cmd_vel_cb, this, std::placeholders::_1));
    _gz_cmdVelPub = _gz_node.Advertise<gz::msgs::Twist>("/cmd_vel");
}
``` 
- En la función Update, publicamos el mensaje. También tenemos que llamar a la función spin: 
  - __Recuerda__: la función spin permite que las callbacks funcionen correctamente. Podemos usar spin_some en el nodo ROS 2 para evitar el comportamiento de bloqueo del spin.
```
void PubCmdVel::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &/*_ecm*/) {    
    rclcpp::spin_some( _ros_node );
    _gz_cmdVelPub.Publish(_cmdVelMsg);
}
``` 
4) Editamos el _CMakeLists.txt_. 
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
  - aqui llamamos el plugin CmdVelPlugin
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
5) Añadimos el plugin al robot _xacro_
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
  - El nombre del plugin es el del CMakelists, la classe es _cmd_vel_plugin::PubCmdVel_
``` 
        <plugin filename="CmdVelPlugin" name="cmd_vel_plugin::PubCmdVel">
        </plugin>
    </gazebo>
``` 
6) Vamos a crear el launch file
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
``` 
    workspace_dir = os.getenv("ROS_WORKSPACE", default=os.getcwd())
    plugin_dir = os.path.join(workspace_dir, "build/cmd_vel_plugin")    

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[plugin_dir]
    )
    
``` 
  - En el bridge, podemos evitar agregar el topic cmd_vel
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

7) Vamos a compilar
``` 
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch cmd_vel_plugin cmd_vel_plugin.launch.py
```
  - Vamos a usar rqt para publicar el cmd_vel topic
```
$ rqt 
```

## Part 2: 
## ros2_control and Gazebo

En esta parte del tutorial, analizaremos el framework ros2_control con dos ejemplos.
- Integración de controladores predeterminados para controlar un brazo robótico
- Desarrollo de un controlador personalizado utilizando ros2_control


Antes de comenzar, necesitamos un robot para probar los controladores. Para ello, implementaremos un modelo de robot de 2 grados de libertad (DOF). Comencemos a crear la estructura, sin los controladores.

1) Vamos a crear el ROS 2 package.
```
$ ros2 pkg create pendulum_description --dependencies xacro
$ cd pendulum_description
$ mkdir urdf
$ mkdir launch
$ touch urdf/pendulum_robot.xacro
$ touch urdf/pendulum_no_controllers.launch.py
```
2) Vamos a editar el modelo del robot: _pendulum_robot.xacro_

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
3) Vamos a crear el launch file (¡Nada de nuevo aquí!)
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
4) Vamos a editar el _CMakeLists.txt_
```
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```
4) Compilamos el workspace y ejecutamos el launch file
```
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch pendulum_description pendulum_no_controllers.launch.py
```

Como se puede observar, el robot cayendo. ¿Por qué?

- Porque no tiene controladores!

#### Add position and state controllers to the robot
- Para permitir el control del robot debemos agregar controladores. La interfaz de control principal es el control de posición, que permite nos especificar la posición de los motores de el robot. 

- El controlador aplicará su mejor esfuerzo para alcanzar la posición deseada. 

- Otra interfaz para controlar los motores son el de velocidad y la fuerza

Para agregar las controladoras sigue estos pasos.

1) Agregue el tag ros2_control al archivo xacro. 

- Esto tag se utiliza para especificar la interfaz de hardware y los juntos da controlar con la interfaz utilizada para el controlo.

- Comencemos desde el archivo xacro anterior, creando uno nuevo llamado: _pendulum_robot_with_controllers.xacro_

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
- Agreguamos el ros2_control tag 
. 
```
<ros2_control name="IgnitionSystem" type="system">
```
- El hardware es la interfaz de ignition con ros2_control. No podemos elegir un controlador diferente

```
<hardware>
  <plugin>ign_ros2_control/IgnitionSystem</plugin>
</hardware>
```
- Solo tenemos un joint, el _revolute_joint_. Vamos a agregarle dos interfaces de control: posición y velocidad.

```
<joint name="revolute_joint">
  <command_interface name="position" />
  <command_interface name="velocity" />
```
- De manera similar, agregamos una interfaz de estado (para publicar el topic _/joint_state_). 
- En este caso, solicitamos la posición y la velocidad de la articulación. 
- Ademas, en la sección de interfaz de posición, podemos especificar la posición inicial de la articulación. En este caso, -1 rad.
```
  <state_interface name="position">
    <param name="initial_value">-1.0</param>
  </state_interface>
  <state_interface name="velocity"/>
</joint>
</ros2_control>
```
2) Agregue el plugin ros2_control

```
<gazebo>
```
- El nombre del archivo y el nombre del complemento son estos para Fortress, para Jazzy este complemento cambia su nombre.
 
```
    <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
```
- Cada controlador incluido aquí debe configurarse. Para configurarlo, utilizamos el archivo yaml clásico.
```    
        <parameters>$(find pendulum_description)/config/pendulum_controller.yaml</parameters>
    </plugin>
</gazebo>
```
3) Edite el archivo de configuración yaml.
- Cree un directorio de configuración en el paquete _pendulum_description_.

```
$ cd src/pendulum_description
$ mkdir config
$ touch config/pendulum_controller.yaml
```
- Aquí está el contenido de el yaml. Es como un yaml clásico que se utiliza para almacenar los parámetros de un nodo.

```
controller_manager:
  ros__parameters:
    update_rate: 100  
```
- Un controlador tiene un nombre. En este caso, lo llamamos position_control. El nombre se utiliza más adelante para especificar parámetros adicionales para el controlador. 
- El tipo de este controlador es _forward_command_controller/ForwardCommandController_. Este controlador simplemente envía los datos de entrada a el motor.
```
    position_control:
      type: forward_command_controller/ForwardCommandController
```
- Hacemos lo mosimo para el _velocity_control_. 
```
    velocity_control:
      type: forward_command_controller/ForwardCommandController
```
- También añadimos el _joint_state_broadcaster_, necesario para publicar el _/joint_states_. Este controlador no necesita configuraciones adicionales, ya que transmite los datos de todas las articulaciones del robot.
```
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```
- Ahora es posible configurar los controladores. Para cada uno de ellos debemos definir los juntos que intervienen en el proceso de control (position/velocity/effort)
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
4) Creamos el launch file para cargar los controles
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
- Cada controlador debe estar cargado y activado para que tenga efecto sobre el motor. 
  - Podemos hacer esto mediante la línea de comandos luego de cargar el modelo del robot. Al mismo tiempo podemos hacerlo en el launch file.
  - Para cargar el controlador, utilizamos la función _ExecuteProcess_ en el archivo de lanzamiento. Aquí ejecutamos el siguiente comando:
        
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
- Cargamos, en estado inactivo los controladores de velocidad, ya que activamos el de posición. 
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
 
        node_robot_state_publisher,
        gz_spawn_entity,
        load_joint_state_broadcaster,
        load_position_controller,
        load_velocity_controller
    ])
```
5) Vamos a editar el _CMakeLists.txt_
```
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```
6) Empeziamos la simulacion
```
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch pendulum_description pendulum_controller.launch.py
```
- Listmao los topics
```
$ ros2 topic list
    /joint_states
    /position_control/commands
    /velocity_control/commands
```
- Controlamos los motore
```
$ ros2 topic pub /posion_control/commands std_msgs/msg/Float64MultiArray "{data: [3]}"
```

7) Interactuar con el controller manager

```
$ ros2 control list_controllers
        list_controllers -> Output the list of loaded controllers, their type and status
```
```
$ ros2 control list_hardware_interfaces
        list_hardware_interfaces -> Output the list of available command and state interfaces

```
8) Cambiar el controlador activo
```
$ ros2 control switch_controllers --activate velocity_control --deactivate position_control
$ ros2 control list_controllers
$ ros2 topic pub /veloty_control/commands std_msgs/msg/Float64MultiArray "{data: [2]}"
```

### Develop a custom controller
A veces resulta útil crear un controlador personalizado para realizar acciones específicas. Definir un controlador específico resulta útil cuando queremos instalar rutinas directamente dentro del controlador. Esto mejora el rendimiento del controlador. Vamos a crear un controlador que controle los comandos angulares de el junto _revolute_joint_ siguiendo un movimiento sinusoidal.

- El perfil de los movimientos sinusoidales se especifica a través de un tema, utilizando un vector


1) Vamos a crear el controller package

```
$ ros2 pkg create sine_ctrl --dependencies rclcpp std_msgs controller_interface pluginlib
$ cd sine_ctrl
$ touch sine_controller.xml
$ src/sine_ctrl.cpp
```

2) Creamos la fuente del controlador
- Al inicio, incluimos los headers para usar la función ROS2, acceder a la interfaz del controlador y manejar los datos de Flota32MultiArray
```
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;
```
- El controlador debe estar implementado dentro de un namespace. Dentro del espacio de nombres, debemos definir la clase del controlador que hereda de la clase base controller_interface::ControllerInterface:
```
namespace sine_controller {
  class SineController : public controller_interface::ControllerInterface
  {
```  
- Entre las diferentes variables, tenemos que proporcionar acceso a la interfaz de estado de los junots, al estado actual de un actuador (posición, velocidad, esfuerzo). El marco ros2_control implementa esto con LoanedStateInterface, que proporciona acceso temporal y controlado a los datos de estado para operaciones en tiempo real, lo que garantiza interacciones seguras y sincronizadas dentro de un controlador. Ayuda a administrar el acceso a los datos de estado sin causar conflictos o inconsistencias en el sistema.

```  
  private:  
    rclcpp::Duration _dt;
```
- Para usar el LoanedStateInterface definimos un alias de tipo para un vector 2D de
std::reference_wrapper<T>. 
En esto caso, la variable _joint_state_interfaces_ usa este tipo para administrar referencias a objetos hardware_interface::LoanedStateInterface:
```     
    template<typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
```    
- Finalmente, se declara un vector de vectores de string para definir el nombre de la interfaz. Debemos considerar un nuevo nombre para cada articulación e interfaz, por ejemplo: _joint_name/position_, _joint_name/velocity_, _joint_name_2/position_, y así sucesivamente:    
```   
   
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
```
- La callback utiliza los datos recibidos para inizializar la amplitud y la frecuencia definidas como miembros de la clase. 
- Primero, verificamos que el formato de los datos sea correcto. Necesitamos 2 números para configurar los parámetros correctamente.
```

    void sine_param_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)  {
    
      if( msg->data.size() != 2 ) {
        cout << "Wrong number of sine parameters" << endl;
        return;
      }
      
      _amplitude = msg->data[0];
      _frequency = msg->data[1];
      
    }

```
- Ahora estamos listos para definir el estado y las interfaces de comandos. Estas funciones son llamadas automáticamente por el ros2_control framework:
```    
    controller_interface::InterfaceConfiguration state_interface_configuration() const {
```    
- Aquí hardcodamos el nombre de las interfaces de estado. La sintaxis es: joint_name/interface. Dado que en la configuración de las articulaciones compartimos la posición y la velocidad como estados de el motor, y tenemos dos articulaciones, agregaremos:
```    
    
      std::vector<std::string> state_interfaces_config_names;
      state_interfaces_config_names.push_back("revolute_joint/position");
      //state_interfaces_config_names.push_back("revolute_joint/velocity");
```
- Para configurar las interfaces de forma efectiva, debemos devolver la lista de interfaces. En este caso, las especificamos mediante la propiedad INDIVIDUAL. Especificamos una lista detallada de las interfaces requeridas, con el formato <joint_name>/<interface_type>:
```
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
    }
```    
Lo mismo ocurre con las interfaces de comandos. Por supuesto, en este caso solo especificamos la posición, ya que queremos controlar la posición de las articulaciones.
```
    controller_interface::InterfaceConfiguration command_interface_configuration() const {
      std::vector<std::string> command_interfaces_config_names;
      command_interfaces_config_names.push_back("revolute_joint/position");
      
```     
Y, además, en este caso, devolvemos la lista de las interfaces INDIVIDUALES que participan en el control del motor:
```      
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces_config_names};
    }
```
- En la función on_configure podemos realizar todas las operaciones necesarias antes de iniciar los controladores. Aquí inicializamos el suscriptor de los parámetros sinusoidales
```
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      
      _sine_param_sub =   get_node()->create_subscription<std_msgs::msg::Float32MultiArray>("/sine_param", 10, 
            std::bind(&SineController::sine_param_cb, this, std::placeholders::_1));
```
- En cuanto a la variable _dt, es lo step de la funcion de update. El parámetro update_rate_ se utiliza para recuperar la velocidad del controlador del servidor de parámetros ROS 2. Como se ve, especificamos los parámetros en el archivo .yaml para configurar los controladores.
```
      _dt = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / update_rate_));

      _amplitude = 0.0;
      _frequency = 0.0;
```
- Luego redimensionamos el comando a los vectores de la interfaz.
```
      command_interfaces_.reserve     (_j_size); 
      state_interfaces_.reserve       (_j_size);
      joint_state_interfaces_.resize  (_j_size);
      state_interface_names_.resize   (_j_size);
```  
- También inicializamos una lista de las interfaces, la usaremos más adelante.
```     
      std::vector<std::string> joint_name = {"revolute_joint"};
      for(int i = 0; i < _j_size; i++) {
        state_interface_names_[i].resize(1);
        state_interface_names_[i][0] = joint_name[i] + "/position";
      }
```
- Esta función debe devolver el SUCCESS o FAIL de la inicialización. Imagina que en esta función necesitas leer algunos parámetros. Si los parámetros no existen o no están especificados correctamente, podrías obtener un FAIL como resultado; sin embargo, en nuestro caso es SUCCESS.
```
      RCLCPP_INFO(get_node()->get_logger(), "configure successful");
      return controller_interface::CallbackReturn::SUCCESS;
    }
```
- Otra función a implementar es la función on_activate. Esta función se llama cuando el controlador pasa de un estado inactivo a uno activo:
```
    // quando il controllore passa in attivo
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
``` 
- Llamamos a la función _get_ordered_interfaces_ desde el espacio de nombres _controller_interface_ que se utiliza para recuperar y organizar una
lista de interfaces, específicamente _state_interfaces_:
``` 
    
      for(int i = 0; i < 1; i++) 
        controller_interface::get_ordered_interfaces( state_interfaces_, state_interface_names_[i], std::string(""),joint_state_interfaces_[i]);
```    
- Luego recuperamos la posición de el junto para guardar el valor inicial del motor     
```         
      _initial_joint_position = joint_state_interfaces_[0][0].get().get_value();
      RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
      t_ = 0.0;

      return controller_interface::CallbackReturn::SUCCESS;
    }

```
- No realizamos ninguna operación en las funciones on_init y deactivate
 ``` 
    // on init viene chiamato quando c' il load del controllore
    controller_interface::CallbackReturn on_init() {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
      return controller_interface::CallbackReturn::SUCCESS;
    }
```    
- Finalmente, podemos escribir la función de update. 
  - Aquí debemos calcular los valores deseados de las articulaciones que siguen el perfil sinusoidal.
```  
    
    controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
 ```
- Primero, incrementamos la variable de tiempo local que se utiliza para procesar la señal sinusoidal. Luego, calculamos la posición deseada del joint con la fórmula sinusoidal clásica: x(t) = Amplitud*sin(2*pi*F*t)
```  
        t_ += _dt.seconds();
        _desired_joint_positions = _initial_joint_position + (_amplitude * std::sin(2 * M_PI * _frequency * t_));
```  
- Finalmente, podemos establecer el valor de posición deseado mediante la interfaz de comandos.
```   

        command_interfaces_[0].set_value(_desired_joint_positions);
        return controller_interface::return_type::OK;
    }
  };
} // namespace sine_controller
```
- Ahora estamos listos para instalar el complemento. Utilizamos la macro __PLUGINLIB_EXPORT_CLASS__. Esta macro está definida en el archivo de encabezado _pluginlib/class_list_macros.hpp_. Por este motivo, debemos incluir este archivo de encabezado en nuestro código. Para exportar correctamente el controlador, debemos especificar como primer parámetro la clase que contiene el controlador.
```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sine_controller::SineController, controller_interface::ControllerInterface)
```

3) Añandir el xml file
```
<library path="sine_controller">
  <class name="sine_controller/SineController" type="sine_controller::SineController" base_class_type="controller_interface::ControllerInterface">
  <description>
    The joint controller commands a group of joints in a given interface
  </description>
  </class>
</library>
```
4) Editar el _CMakeLists.txt_
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
5) Añadir el controller al robot model
```
find_package(sine_ctrl REQUIRED)
```
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
6) Empezamos el controller
```
$ ros2 control switch_controllers --activate sine_controller --deactivate position_control
$ ros2 topic pub /sine_param std_msgs/msg/Float32MultiArray "{data: [1.3, 0.3]}"
```

