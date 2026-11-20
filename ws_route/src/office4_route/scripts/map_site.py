"""Per-map storage for the Jetson mapping and route commands.

This file is the ONLY place that decides which map is in force. Everything else
(bash scripts, launch files, dogroute) asks here.

Resolution order
----------------
1. ``DOG_ROUTE_MAP`` environment variable   (set by dogroute / systemd units)
2. ``~/.config/dogroute/selected-map``      (written by ``dogroute use <name>``)
3. legacy fallback ``office4``              (only if that directory exists)
"""
import hashlib
import json
import math
import os
import re
import shlex
import sys
from pathlib import Path

MAPS = Path.home() / 'robot_ws/maps'
STATE = Path.home() / '.config/dogroute/selected-map'
LEGACY = 'office4'


def map_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value):
        raise ValueError('地图名使用 1～64 个字母、数字、下划线或短横线，首字符用字母或数字。')
    return value


def saved_name():
    """Map name stored by ``dogroute use``, or None."""
    if not STATE.is_file():
        return None
    raw = STATE.read_text().strip()
    return map_name(raw) if raw else None


def resolve(quiet=True):
    """Map name in force for this process."""
    name = os.environ.get('DOG_ROUTE_MAP') or saved_name()
    if name:
        return map_name(name)
    if (MAPS / LEGACY).is_dir():
        if not quiet:
            print(f'[提示] 还没选过地图，暂时沿用 {LEGACY}；切图请执行 dogroute use <名字>',
                  file=sys.stderr)
        return LEGACY
    raise SystemExit('还没有选择地图：先执行 dogroute use <地图名>（dogroute status 可查看现状）。')


def map_paths(name=None):
    """(directory, map.pcd, nav2/map.yaml, routes/hand_drawn.json)"""
    directory = MAPS / (name or resolve())
    return (directory, directory / 'map.pcd', directory / 'nav2/map.yaml',
            directory / 'routes/hand_drawn.json')


MAP_NAME = resolve()
MAP_DIR = MAPS / MAP_NAME


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


def shell_exports(name=None):
    """Shell code that pins this process to one map (evaluated by dog_map.bash)."""
    name = map_name(name) if name else resolve(quiet=False)
    directory, pcd, grid, route = map_paths(name)
    return '\n'.join([
        f'export DOG_ROUTE_MAP={shlex.quote(name)}',
        f'export DOG_MAP_DIR={shlex.quote(str(directory))}',
        f'export DOG_MAP_PCD={shlex.quote(str(pcd))}',
        f'export DOG_MAP_GRID={shlex.quote(str(grid))}',
        f'export DOG_ROUTE_JSON={shlex.quote(str(route))}',
        f'export DOG_LEGACY_MAP={shlex.quote(LEGACY)}',
    ])


def describe(name=None):
    name = map_name(name) if name else resolve()
    directory, pcd, grid, route = map_paths(name)
    return {'map': name, 'dir': str(directory), 'pcd': str(pcd), 'grid': str(grid),
            'route': str(route), 'has_dir': directory.is_dir(), 'has_pcd': pcd.is_file(),
            'has_grid': grid.is_file(), 'has_route': route.is_file()}


def main(argv):
    import argparse
    parser = argparse.ArgumentParser(description='解析当前地图（被 dogroute 与所有 bash 脚本共用）')
    parser.add_argument('--shell', action='store_true', help='输出可 eval 的 export 语句')
    parser.add_argument('--json', action='store_true', help='输出 JSON')
    parser.add_argument('--name', action='store_true', help='只输出地图名')
    parser.add_argument('--list', action='store_true', help='列出所有地图及文件齐备情况')
    parser.add_argument('--check', action='store_true', help='检查当前地图是否可循线（缺文件则退出非 0）')
    parser.add_argument('--map', metavar='NAME', help='指定地图名（默认按解析顺序）')
    args = parser.parse_args(argv)

    if args.list:
        for directory in sorted(p for p in MAPS.iterdir() if p.is_dir() and not p.name.startswith('.')):
            flags = [tag if (directory / rel).exists() else '-' for tag, rel in
                     (('pcd', 'map.pcd'), ('grid', 'nav2/map.yaml'),
                      ('route', 'routes/hand_drawn.json'))]
            mark = '*' if directory.name == resolve() else ' '
            print(f'{mark} {directory.name:<16} {" ".join(flags)}')
        return 0
    if args.name:
        print(map_name(args.map) if args.map else resolve())
        return 0
    if args.json:
        print(json.dumps(describe(args.map), indent=2, ensure_ascii=False))
        return 0
    if args.check:
        info = describe(args.map)
        missing = [key for key in ('has_dir', 'has_pcd', 'has_route') if not info[key]]
        if missing:
            print(f'{info["map"]} 还不能循线，缺少：{", ".join(missing)}', file=sys.stderr)
            return 1
        print(info['map'])
        return 0
    print(shell_exports(args.map))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
