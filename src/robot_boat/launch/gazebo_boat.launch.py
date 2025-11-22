import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Ruta al URDF
    urdf_file = os.path.join(
        get_package_share_directory('robot_boat'),
        'urdf',
        'robot_boat.urdf'
    )
    
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    return LaunchDescription([
        # Publicar el URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # Iniciar Gazebo primero
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # Spawn del robot después de que Gazebo esté listo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'robot_boat', '-file', urdf_file],
            output='screen'
        ),
        
        # Cliente de Gazebo
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        )
    ])
