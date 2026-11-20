"""Point cloud and route geometry for the selected PGO map."""
import hashlib
import math
import os
import json
from pathlib import Path as File
from tempfile import NamedTemporaryFile

import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField

from map_site import MAP_DIR, MAP_NAME
ROUTE = MAP_DIR / 'routes/hand_drawn.json'


def read_map():
    with (MAP_DIR / 'map.pcd').open('rb') as f:
        header = {}
        for line in f:
            fields = line.decode('ascii').split()
            if fields:
                header[fields[0]] = fields[1:]
            if fields[:1] == ['DATA']:
                break
        if (header.get('FIELDS') != ['x','y','z','intensity'] or
                header.get('DATA') != ['binary'] or header.get('SIZE') != ['4'] * 4):
            raise ValueError('Expected binary XYZI PGO PCD')
        payload = f.read()
        points = np.frombuffer(payload, '<f4').reshape(-1, 4)[:, :3]
        if len(points) != int(header['POINTS'][0]):
            raise ValueError('Incomplete PGO PCD')
        return points[np.isfinite(points).all(axis=1)], hashlib.sha256(payload).hexdigest()


def xyz(cloud):
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [('>' if cloud.is_bigendian else '<')+'f4']*3,
                      'offsets': [next(f.offset for f in cloud.fields if f.name==key and f.datatype==7)
                                  for key in ('x','y','z')], 'itemsize': cloud.point_step})
    view = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                      strides=(cloud.row_step, cloud.point_step))
    p = np.column_stack([view[key].ravel() for key in dtype.names])
    return p[np.isfinite(p).all(axis=1)]


def cloud_message(header, points):
    c = PointCloud2()
    c.header = header
    c.height, c.width = 1, len(points)
    c.point_step, c.row_step = 12, 12*len(points)
    c.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i,k in enumerate('xyz')]
    c.is_dense = True
    c.data = np.asarray(points, dtype='<f4').tobytes()
    return c


def route_message(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('A route needs at least two finite XY points')
    edges = np.diff(points,axis=0)
    lengths = np.linalg.norm(edges, axis=1)
    if lengths.sum()<0.15 or lengths.sum()>300 or len(points)>30000:
        raise ValueError('Invalid stroke length')
    path = Path()
    path.header.frame_id = 'map'
    for a,delta,length in zip(points,edges,lengths):
        if length < 1e-5:
            continue
        yaw = math.atan2(delta[1],delta[0])
        subdivisions = max(1,math.ceil(length/0.05))
        for j in range(subdivisions):
            x,y = a + delta*(j/subdivisions)
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y = float(x),float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
            path.poses.append(p)
    last = PoseStamped()
    last.header.frame_id = 'map'
    last.pose.position.x,last.pose.position.y = map(float,points[-1])
    last.pose.orientation = path.poses[-1].pose.orientation
    path.poses.append(last)
    return path


def save_route(dest, points, fingerprint):
    dest.parent.mkdir(parents=True,exist_ok=True)
    with NamedTemporaryFile('w',dir=dest.parent,delete=False) as f:
        try:
            json.dump({'map':MAP_NAME,'frame':'map','pcd_sha256':fingerprint,'xy':points},f)
            f.flush()
            os.fsync(f.fileno())
            os.replace(f.name,dest)
        finally:
            File(f.name).unlink(missing_ok=True)


def load_route(dest, fingerprint):
    data=json.loads(dest.read_text())
    if (data['map'],data['frame'],data['pcd_sha256']) != (MAP_NAME,'map',fingerprint):
        raise ValueError('Saved route belongs to a different map')
    return route_message(data['xy'])
