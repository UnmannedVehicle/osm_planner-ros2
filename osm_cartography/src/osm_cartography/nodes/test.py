#!/usr/bin/env python

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



import os
import uuid

from xml.etree import ElementTree

from geodesy import bounding_box

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import MapFeature
from geographic_msgs.msg import WayPoint
from unique_identifier_msgs.msg import UUID



def get_required_attribute(el, key):
    """
    Get attribute key of element *el*.

    :raises:  :exc:`ValueError` if key not found
    """
    val = el.get(key)
    if val is None:
        raise ValueError('required attribute missing: ' + key)
    return val


def make_osm_unique_id(namespace, el_id):
    """
    Make UUID message for *el_id* number in OSM sub-namespace *namespace*.

    :param namespace: OSM sub-namespace
    :type  namespace: string
    :param el_id: OSM identifier within that namespace
    :type  el_id: int or string containing an integer

    :returns: corresponding `unique_identifier_msgs/UUID`_ message.
    :raises:  :exc:`ValueError`
    """
    if namespace not in {'node', 'way', 'relation'}:
        raise ValueError('invalid OSM namespace: ' + namespace)
    ns = 'http://openstreetmap.org/' + namespace + '/'
    return UUID(uuid=list(uuid.uuid5(uuid.NAMESPACE_URL, ns + str(el_id)).bytes))


def get_tag(el):
    """
    :returns: `geographic_msgs/KeyValue`_ message for `<tag>` *el* if any, None otherwise.
    """
    pair = None
    key = el.get('k')
    if key is not None:
        pair = KeyValue()
        pair.key = key
        pair.value = get_required_attribute(el, 'v')
        return pair


def get_osm(url, bounds):
    """Get `geographic_msgs/GeographicMap`_ from Open Street Map XML data.

    The latitude and longitude of the bounding box returned may differ
    from the requested bounds.

    :param url:    Uniform Resource Locator for map.
    :param bounds: Desired `geographic_msgs/BoundingBox`_ for map (presently ignored).
    :returns: `geographic_msgs/GeographicMap`_ message (header not filled in).
    """
    # parse the URL
    if url.startswith('file:///'):
        filename = url[7:]
    elif url.startswith('package://'):
        pkg_name, slash, pkg_path = url[10:].partition('/')
        filename = SHARE_DIR + '/' + pkg_path
    else:
        raise ValueError('unsupported URL: ' + url)

    gmap = GeographicMap(id=UUID(uuid=list(uuid.uuid5(uuid.NAMESPACE_URL, url).bytes)))

    xm = None
    try:
        with open(filename, 'r') as f:
            xm = ElementTree.parse(f)
    except IOError:
        raise ValueError('unable to read ' + str(url))
    except ElementTree.ParseError:
        raise ValueError('XML parse failed for ' + str(url))
    osm = xm.getroot()

    # get map bounds
    for el in osm.iterfind('bounds'):
        minlat = float(get_required_attribute(el, 'minlat'))
        minlon = float(get_required_attribute(el, 'minlon'))
        maxlat = float(get_required_attribute(el, 'maxlat'))
        maxlon = float(get_required_attribute(el, 'maxlon'))
        gmap.bounds = bounding_box.makeBounds2D(minlat, minlon, maxlat, maxlon)

    # get map way-point nodes
    for el in osm.iterfind('node'):

        way = WayPoint()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('node id missing')
        way.id = make_osm_unique_id('node', el_id)

        way.position.latitude = float(get_required_attribute(el, 'lat'))
        way.position.longitude = float(get_required_attribute(el, 'lon'))
        way.position.altitude = float(el.get('ele', float('nan')))

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv is not None:
                way.props.append(kv)

        gmap.points.append(way)

    # get map paths
    for el in osm.iterfind('way'):
        feature = MapFeature()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('way id missing')
        feature.id = make_osm_unique_id('way', el_id)

        for nd in el.iterfind('nd'):
            way_id = get_required_attribute(nd, 'ref')
            feature.components.append(make_osm_unique_id('node', way_id))

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv is not None:
                feature.props.append(kv)

        gmap.features.append(feature)

    # get relations
    for el in osm.iterfind('relation'):

        feature = MapFeature()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('relation id missing')
        feature.id = make_osm_unique_id('relation', el_id)

        for mbr in el.iterfind('member'):
            mbr_type = get_required_attribute(mbr, 'type')
            if mbr_type in {'node', 'way', 'relation'}:
                mbr_id = get_required_attribute(mbr, 'ref')
                feature.components.append(make_osm_unique_id(mbr_type, mbr_id))
            else:
                print('unknown relation member type: ' + mbr_type)

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv is not None:
                feature.props.append(kv)

        gmap.features.append(feature)

    return gmap


interesting_tags = {'access',
                    'amenity',
                    'boundary',
                    'bridge',
                    'building',
                    'ele',
                    'highway',
                    'landuse',
                    'lanes',
                    'layer',
                    'maxheight',
                    'maxspeed',
                    'maxwidth',
                    'name',
                    'network',
                    'oneway',
                    'railway',
                    'ref',
                    'restriction',
                    'route',
                    'street',
                    'tunnel',
                    'type',
                    'width'}

ignored_values = {'bridleway',
                  'construction',
                  'cycleway',
                  'footway',
                  'path',
                  'pedestrian',
                  'proposed',
                  'steps'}
                  
                  
class VizNode():
    def __init__(self):
        """
        ROS node to publish visualization markers for a GeographicMap.
        """
        
        map_url = "file:///home/zxd/tests/map.osm"
        # advertise visualization marker topic
        
        # refresh the markers every three seconds, making them last four.
        
        self.map = get_osm(map_url, bounding_box.makeGlobal())
        self.get_markers(self.map)

    def get_markers(self, gmap):
        """Get markers for a GeographicMap message.

        :post: self.msg = visualization markers message
        """
        self.map = gmap

        self.map_points = geodesy.wu_point.WuPointSet(gmap.points)
        self.msg = MarkerArray()
        self.mark_way_points(ColorRGBA(r=1., g=1., b=0., a=0.8))




    def mark_way_points(self, color):
        """Create slightly transparent disks for way-points.

        :param color: disk RGBA value
        """
        index = 0
        for wp in self.map_points:

            index += 1
            print(index)
            # use easting and northing coordinates (ignoring altitude)
           
            #self.msg.markers.append(marker)
    


if __name__ == '__main__':
    # run main function and exit
    viznode = VizNode()
