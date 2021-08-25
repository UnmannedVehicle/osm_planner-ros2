#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
Create rviz markers for geographic information maps from Open Street
Map server.
"""


import sys
import math
import geodesy.props
import geodesy.utm
import geodesy.wu_point
from geodesy import bounding_box

from geographic_msgs.msg import GeoPoint
from geographic_msgs.srv import GetGeographicMap
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from osm_cartography import xml_map

class VizNode(Node):
    def __init__(self):
        """
        ROS node to publish visualization markers for a GeographicMap.
        """
        super().__init__("viz_osm")
        self.config = None
        map_url = self.declare_parameter("map_url").value
        # advertise visualization marker topic
        self.pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)      
        self.map = None
        self.msg = None
        # refresh the markers every three seconds, making them last four.
        self.timer_interval = 3
        self.marker_life = rclpy.duration.Duration(seconds=self.timer_interval + 1).to_msg()
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)
                
        self.map = xml_map.get_osm(map_url, bounding_box.makeGlobal())
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.header.frame_id = '/world'
        self.get_markers(self.map)

    def get_markers(self, gmap):
        """Get markers for a GeographicMap message.

        :post: self.msg = visualization markers message
        """
        self.map = gmap

        self.map_points = geodesy.wu_point.WuPointSet(gmap.points)
        #self.get_logger().info(f"id:{self.map}")
        self.msg = MarkerArray()
        self.mark_boundaries(ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8))
        self.mark_way_points(ColorRGBA(r=1., g=1., b=0., a=0.8))

        # define arguments for displaying various feature types
        road_props = {'bridge', 'highway', 'tunnel'}
        fargs = [(lambda f: geodesy.props.match(f, road_props),
                  ColorRGBA(r=8., g=0.2, b=0.2, a=0.8),
                  "roads_osm"),
                 (lambda f: geodesy.props.match(f, {'building'}),
                  ColorRGBA(r=0., g=0.3, b=0.7, a=0.8),
                  "buildings_osm"),
                 (lambda f: geodesy.props.match(f, {'railway'}),
                  ColorRGBA(r=0., g=0.7, b=.7, a=0.8),
                  "railroad_osm"),
                 (lambda f: geodesy.props.match(f, {'amenity', 'landuse'}),
                  ColorRGBA(r=0., g=1., b=0., a=0.5),
                  "other_osm")]
        for args in fargs:
            self.mark_features(*args)

    def mark_boundaries(self, color):
        # draw outline of map boundaries
        marker = Marker(header=self.map.header,
                        ns="bounds_osm",
                        id=0,
                        type=Marker.LINE_STRIP,
                        action=Marker.ADD,
                        scale=Vector3(x=2.),
                        color=color,
                        lifetime=self.marker_life)

        # Convert bounds latitudes and longitudes to UTM (no
        # altitude), convert UTM points to geometry_msgs/Point
        bbox = self.map.bounds
        min_lat, min_lon, max_lat, max_lon = bounding_box.getLatLong(bbox)
        p0 = geodesy.utm.fromLatLong(min_lat, min_lon).toPoint()
        p1 = geodesy.utm.fromLatLong(min_lat, max_lon).toPoint()
        p2 = geodesy.utm.fromLatLong(max_lat, max_lon).toPoint()
        p3 = geodesy.utm.fromLatLong(max_lat, min_lon).toPoint()

        # add line strips to bounds marker
        marker.points.append(p0)
        marker.points.append(p1)
        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p3)
        marker.points.append(p0)
        self.msg.markers.append(marker)

    def mark_features(self, predicate, color, namespace):
        """
        Create outline for map features

        :param predicate: function to match desired features
        :param color: RGBA value
        :param namespace: Rviz namespace.

        :todo: differentiate properties for: highway, building,
               bridge, tunnel, amenity, etc.
        """
        zone =49
        index = 0
        for feature in filter(predicate, self.map.features):
            marker = Marker(header=self.map.header,
                            ns=namespace,
                            id=index,
                            type=Marker.LINE_STRIP,
                            action=Marker.ADD,
                            scale=Vector3(x=2.),
                            color=color,
                            lifetime=self.marker_life)
            index += 1
            prev_point = None
            for mbr in feature.components:
                wu_point = self.map_points.get(str(mbr.uuid))
                if wu_point:  # this component is a way point
                    wu_point.utm = geodesy.utm.UTMPoint 
                    lat = wu_point.way_pt.position.latitude
                    lon = wu_point.way_pt.position.longitude
                    x,y = self.LatLonToXY(lat,lon,zone)            
                    wu_point.utm.easting = x
                    wu_point.utm.northing = y
                    wu_point.utm.zone = zone
                    wu_point.band = 'R'
                    p = wu_point.toPointXY()
                    if prev_point:
                        marker.points.append(prev_point)
                        marker.points.append(p)
                    prev_point = p
            self.msg.markers.append(marker)

    def mark_way_points(self, color):
        """Create slightly transparent disks for way-points.

        :param color: disk RGBA value
        """        
        index = 0
        z = 0
        
        for wp in self.map_points:
            marker = Marker(header=self.map.header,
                            ns="waypoints_osm",
                            id=index,
                            type=Marker.CYLINDER,
                            action=Marker.ADD,
                            scale=Vector3(x=2., y=2., z=0.2),
                            color=color,
                            lifetime=self.marker_life)
            index += 1
            wp.utm = geodesy.utm.UTMPoint 
            lat = wp.way_pt.position.latitude
            lon = wp.way_pt.position.longitude
            x,y = self.LatLonToXY(lat,lon,int(lon / 6.0) + 31)            
            wp.utm.easting = x
            wp.utm.northing = y
            wp.utm.zone = int(lon / 6.0) + 31
            wp.utm.band = 'R'
            
            
            
            #self.get_logger().info(f"id:{type(wp.utm)}")
            # use easting and northing coordinates (ignoring altitude)
            marker.pose.position = wp.toPointXY()
            marker.pose.orientation = Quaternion(x=0., y=0., z=0., w=1.)
            self.msg.markers.append(marker)


    def LatLonToXY(self,lat,lon,zone):
        sm_a = 6378137.0
        sm_b = 6356752.314
        #sm_EccSquared = 6.69437999013e-03;
        UTMScaleFactor = 0.9996;
        pi = 3.14159265358979
        lambda0 = (-183.0 + (zone * 6.0)) / 180.0 * pi
        phi = lat / 180.0 * pi
        lambda1 = lon / 180.0 * pi

        ep2 = (math.pow(sm_a, 2.0) - math.pow(sm_b, 2.0)) / math.pow(sm_b, 2.0)	
        nu2 = ep2 * math.pow(math.cos(phi), 2.0)	
        N = math.pow(sm_a, 2.0) / (sm_b * math.sqrt(1 + nu2))
        t = math.tan (phi)
        t2 = t * t
        tmp = (t2 * t2 * t2) - math.pow(t, 6.0)
        l = lambda1 - lambda0
        l3coef = 1.0 - t2 + nu2
        l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2)
        l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2
        l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2	- 330.0 * t2 * nu2
        l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2)
        l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2)	
        x = N * math.cos (phi) * l + (N / 6.0 * math.pow(math.cos(phi), 3.0) * l3coef * math.pow(l, 3.0))+ (N / 120.0 * math.pow(math.cos(phi), 5.0) * l5coef * math.pow(l, 5.0))+ (N / 5040.0 * math.pow(math.cos (phi), 7.0) * l7coef * math.pow(l, 7.0))


        n = (sm_a - sm_b) / (sm_a + sm_b)	
        alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (math.pow(n, 2.0) / 4.0) + (math.pow(n, 4.0) / 64.0))
        beta = (-3.0 * n / 2.0) + (9.0 * math.pow(n, 3.0) / 16.0) + (-3.0 * math.pow(n, 5.0) / 32.0)
        gamma = (15.0 *math.pow(n, 2.0) / 16.0) + (-15.0 * math.pow(n, 4.0) / 32.0)
        delta = (-35.0 * math.pow(n, 3.0) / 48.0) + (105.0 * math.pow(n, 5.0) / 256.0)
        epsilon = (315.0 * math.pow(n, 4.0) / 512.0);
        result = alpha * (phi + (beta * math.sin(2.0 * phi)) + (gamma * math.sin(4.0 * phi)) + (delta * math.sin(6.0 * phi)) + (epsilon * math.sin(8.0 * phi)))

        y = result+ (t / 2.0 * N * math.pow(math.cos(phi), 2.0) * math.pow(l, 2.0))+ (t / 24.0 * N * math.pow(math.cos(phi), 4.0) * l4coef * math.pow(l, 4.0))+ (t / 720.0 * N * math.pow(math.cos(phi), 6.0) * l6coef * math.pow(l, 6.0))+ (t / 40320.0 * N * math.pow(math.cos(phi), 8.0) * l8coef * math.pow(l, 8.0))

        x = x * UTMScaleFactor + 500000.0
        y = y * UTMScaleFactor
        if (y < 0.0):
            y += 10000000.0
        return x,y
 

    def timer_callback(self):
        """
        Called periodically to refresh map visualization.
        """
        #self.get_logger().info(f"publish")
        if self.msg is not None:
            now = self.get_clock().now().to_msg()
            for m in self.msg.markers:
                m.header.stamp = now
            self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    viznode = VizNode()
    try:
    	while rclpy.ok():
        	rclpy.spin_once(viznode)
    except rclpy.exceptions.ROSInterruptException:
        pass


if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
