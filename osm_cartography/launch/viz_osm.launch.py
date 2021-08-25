   
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    #map_url = "package://osm_cartography/tests/map.osm	"
    map_url = "file://"+os.path.join(get_package_share_directory("osm_cartography"), "tests", "map.osm")
    #map_url = "file:///home/zxd/tests/map.osm"
    rviz_config_dir = os.path.join(
        get_package_share_directory('osm_plan'),
        'rviz',
        'osm.rviz')
        


    return LaunchDescription([

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
