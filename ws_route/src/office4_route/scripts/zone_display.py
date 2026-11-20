"""Draw the same polygons used for counting, including disconnected pieces and holes."""
import numpy as np
from geometry_msgs.msg import Point
from shapely.geometry import LineString
from visualization_msgs.msg import Marker, MarkerArray


def lines(geometry):
    if geometry.is_empty:
        return
    if geometry.geom_type == 'Polygon':
        yield geometry.exterior
        yield from geometry.interiors
    elif geometry.geom_type in ('LineString', 'LinearRing'):
        yield geometry
    elif hasattr(geometry, 'geoms'):
        for part in geometry.geoms:
            yield from lines(part)


def outline(geometry, name, rgb, width, stamp=None, hatch=False):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.ns = name
    marker.type = Marker.LINE_LIST
    marker.pose.orientation.w = 1.
    marker.scale.x = width
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = 1.
    if stamp is not None:
        marker.header.stamp = stamp
        marker.lifetime.nanosec = 900000000
    if geometry.is_empty:
        marker.action = Marker.DELETE
        return marker
    strokes = list(lines(geometry))
    if hatch:
        left, bottom, right, top = geometry.bounds
        for y in np.arange(bottom + .025, top, .05):
            segment = LineString([(left - .01, y), (right + .01, y)])
            strokes.extend(lines(geometry.intersection(segment)))
    for stroke in strokes:
        coords = list(stroke.coords)
        for a, b in zip(coords, coords[1:]):
            marker.points.extend([Point(x=x, y=y, z=.10) for x, y in (a, b)])
    return marker


def active_markers(box, overlap, stamp):
    return MarkerArray(markers=[
        outline(box, 'front_box', (.25, .55, 1.), .014, stamp),
        outline(overlap, 'intersection', (.30, 1., .32), .009, stamp, hatch=True)])
