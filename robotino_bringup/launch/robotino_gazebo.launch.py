import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    robotino_description_path = os.path.join(
        get_package_share_directory('robotino_description'))

    xacro_file = os.path.join(robotino_description_path,
                              'urdf',
                              'robotino.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    rviz_config_path = os.path.join(get_package_share_directory('robotino_bringup'),
                             'rviz', 'urdf_config.rviz')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    ) 
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')

    '''
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["omnidirectional_controller"],
    )
    '''
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]

    )

    return LaunchDescription([
        # joint_state_broadcaster_spawner,
        # robot_controller_spawner,
        gazebo,
        spawn_entity,
        node_robot_state_publisher,
        rviz2_node,
    ])