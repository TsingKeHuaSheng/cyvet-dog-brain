"""Per-map storage for the Jetson mapping and route commands."""
import hashlib
import json
import math
import os
import re
from pathlib import Path


MAPS = Path.home() / 'robot_ws/maps'


def map_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value):
        raise ValueError('地图名使用 1～64 个字母、数字、下划线或短横线，首字符用字母或数字。')
    return value


def floor_height(directory):
    for record in (directory / 'nav2/projection.json', directory / 'navigation_preview/projection.json'):
        if record.is_file():
            projection = json.loads(record.read_text())
            if projection.get('source') == str(directory.resolve()):
                floor = float(projection['floor_z'])
                if math.isfinite(floor):
                    return floor
    # The first PGO pose is the body IMU in map coordinates. This estimate
    # assumes the normal standing mapping posture on a level starting surface.
    from ament_index_python.packages import get_package_share_directory
    ref = Path(get_package_share_directory('localizer')) / 'config/base_link_reference.json'
    model = json.loads(ref.read_text())
    height = model['assembled_robot_m']['mount_center_height_standing']
    height += model['lidar_mount_extrinsic']['mount_center_to_body_imu_m'][2]
    with (directory / 'poses.txt').open() as stream:
        first = next((line.split() for line in stream if line.strip()), [])
    if len(first) != 8:
        raise ValueError('poses.txt 缺少第一帧，无法推算地面高度。')
    return round(float(first[3]) - height, 5)


def pcd_digest(directory):
    with (directory / 'map.pcd').open('rb') as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


MAP_NAME = map_name(os.environ.get('DOG_ROUTE_MAP', 'office4'))
MAP_DIR = MAPS / MAP_NAME
