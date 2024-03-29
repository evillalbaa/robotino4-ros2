import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'turtlebot_area.yaml')

    rviz_config_path = os.path.join(get_package_share_directory('map_server'),
                             'rviz', 'map.rviz')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename':map_file} ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )


    return LaunchDescription([
        #rviz2_node,
        map_server_node,
        lifecycle_manager_node,
       
    ])