
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    map_file_name = 'map.osm'
    origin_latitude = 28.185424
    origin_longitude = 112.945468
    
    map_url = "file://"+os.path.join(get_package_share_directory("osm_cartography"), "tests", "map.osm")
    
    osm_map_path = os.path.join(
        get_package_share_directory('osm_plan'),
        'osm_example',
        map_file_name)
   
    rviz_config_dir = os.path.join(
        get_package_share_directory('osm_plan'),
        'rviz',
        'osm.rviz')
        
    param_config_dir = os.path.join(
        get_package_share_directory('osm_plan'),
        'config',
        'ros_param.yaml')

    return LaunchDescription([

        Node(
            package='osm_plan',
            executable='osm_planner',
            name='osm_planner',
            output='screen',
            parameters=[{'osm_map_path': osm_map_path,
            		  'origin_latitude':origin_latitude,
            		  'origin_longitude':origin_longitude}],
            ),

        Node(
            package='osm_plan',
            executable='utm',
            output='screen',
            parameters=[{'origin_latitude': origin_latitude,
            		  'origin_longitude':origin_longitude,}],
			),
			
        Node(
            package='osm_cartography',
            executable='viz_osm',
            name='viz_osm',
            output='screen',
            parameters=[{'map_url': map_url}],
            ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['690975', '3119274', '0', '0', '0', '0', 'world', 'local_map']
        ),
       
       
       Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            ),               
        

    ])
