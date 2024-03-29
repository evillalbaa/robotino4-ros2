import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    rviz_config_path = os.path.join(get_package_share_directory('cartographer_slam'),
                             'rviz', 'map.rviz')
    robotiono_world = os.path.join(get_package_share_directory('robotino_bringup'), 'launch/robotino_world.xml')
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    gazebo_launch_file = IncludeLaunchDescription(XMLLaunchDescriptionSource(robotiono_world))

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        gazebo_launch_file,
        rviz2_node,
        cartographer_node,
        occupancy_grid_node,
       
    ])