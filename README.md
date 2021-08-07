# osm-planner

## Parameter initialization
### Select Map
Change map parameter and 'lon', 'lat' in lanuch file.
```
map_file_name = 'map.osm'
origin_latitude = 28.185424
origin_longitude = 112.945468
``` 
### Determine the origin of map coordinates
View latitude and longitude data in "map.osm" file.
```
<bounds minlat="28.1825200" minlon="112.9419600" maxlat="28.1878000" maxlon="112.9489900"/>
```
Then Convert latitude and longitude data to UTM coordinates in "https://www.earthpoint.us/Convert.aspx".
And set TF conversion in launch.
```
<node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="690975 3119274 0 -3.1 0 0 map local_map 100"/>
```
## Test
First,run planner_node and rviz.
```
ros2 launch osm_plan planner_node_launch.py 
``` 
