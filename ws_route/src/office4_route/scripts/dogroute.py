"""One map selection for PGO capture, occupancy grids, localization and route tracking."""
import argparse
import fcntl
import json
import math
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from map_site import LEGACY, MAPS, floor_height, map_name, pcd_digest, saved_name, shell_exports

HOME_DIR = Path.home()
TOOLS = HOME_DIR / 'robot_tools'
SCRIPTS = Path(__file__).resolve().parent
STATE = HOME_DIR / '.config/dogroute'
SELECTED = STATE / 'selected-map'
MAPPER = 'dogroute-mapping'
LOCALIZER = 'office4-localization'
DRAW = 'office4-route'


def running(unit):
    return subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit]).returncode == 0


def service_map(unit):
    env = subprocess.check_output(['systemctl', '--user', 'show', unit, '-p', 'Environment', '--value'], text=True)
    return next((v.split('=', 1)[1] for v in shlex.split(env) if v.startswith('DOG_ROUTE_MAP=')), None)


def unit_map(unit):
    """Map a running unit was started with; units started before this change recorded none."""
    return service_map(unit) or LEGACY


def stop(unit):
    if running(unit):
        subprocess.run(['systemctl', '--user', 'stop', unit], check=True)


def launch(unit, name, workspace, pkg, filename):
    cmd = ['systemd-run', '--user', '--collect', '--unit', unit, f'--setenv=DOG_ROUTE_MAP={name}',
           '--setenv=ROS_DOMAIN_ID=0', '--setenv=OPENBLAS_NUM_THREADS=1',
           '--setenv=DISPLAY=' + os.environ.get('DISPLAY', ':0'),
           '--setenv=XAUTHORITY=' + os.environ.get('XAUTHORITY', f'/run/user/{os.getuid()}/gdm/Xauthority'),
           '/bin/bash', '-c', 'source "$1" && exec ros2 launch "$2" "$3"', 'dogroute',
           str(HOME_DIR / workspace / 'install/setup.bash'), pkg, filename]
    subprocess.run(cmd, check=True)


def call(service, kind, request, *, timeout=20):
    import rclpy
    if not rclpy.ok():
        rclpy.init(args=[])
    node = rclpy.create_node('dogroute_cli')
    try:
        client = node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=timeout):
            raise ValueError(f'等待 {service} 超时。')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise ValueError(f'{service} 尚未确认完成。')
        return future.result()
    finally:
        node.destroy_node()


def map_input_empty(directory):
    if directory.exists() and any(directory.iterdir()):
        raise ValueError(f'{directory} 已有文件。新建图请用另一个地图名。')


def check_no_external_slam():
    proc = subprocess.run(['pgrep', '-f', r'^\S*/(lio_node|pgo_node|localizer_node)( |$)'],
                          stdout=subprocess.DEVNULL)
    if proc.returncode == 0:
        raise ValueError('已有单独启动的 SLAM 节点。请在它的终端结束该任务，再启动建图或定位。')


def mapping(directory):
    if running(MAPPER):
        if unit_map(MAPPER) == directory.name:
            print(f'正在采集 {directory.name}，走完后执行 dogroute save。')
            return
        raise ValueError('已有另一场建图，请先保存。')
    map_input_empty(directory)
    stop(DRAW)
    stop(LOCALIZER)
    check_no_external_slam()
    launch(MAPPER, directory.name, 'ws_slam', 'pgo', 'pgo_launch.py')
    print(f'正在启动 {directory.name} 建图。遥控走一圈，回到起点后执行 dogroute save。')


def save(directory):
    from interface.srv import SaveMaps
    if not running(MAPPER) or unit_map(MAPPER) != directory.name:
        raise ValueError('没有以当前地图名启动的建图任务。')
    map_input_empty(directory)
    MAPS.mkdir(parents=True, exist_ok=True)
    # SaveMaps clears an existing patches folder. Always give it a fresh staging directory.
    staging = Path(tempfile.mkdtemp(prefix=f'.{directory.name}-capture-', dir=MAPS))
    request = SaveMaps.Request(file_path=str(staging), save_patches=True)
    complete = call('/pgo/save_maps', SaveMaps, request, timeout=180)
    if not complete.success:
        shutil.rmtree(staging)
        raise ValueError(complete.message)
    poses = (staging / 'poses.txt').read_text().splitlines()
    if (not poses or (staging / 'map.pcd').stat().st_size == 0 or
            not all((staging / 'patches' / line.split()[0]).is_file() for line in poses if line.strip())):
        raise ValueError(f'保存结果不完整，请重试。建图仍在运行，文件在 {staging}。')
    map_input_empty(directory)
    staging.rename(directory)
    stop(MAPPER)
    print(f'已保存 {len(poses)} 个关键帧到 {directory}，建图已结束。')


def grid(directory, floor):
    if not (directory / 'map.pcd').is_file():
        raise ValueError(f'没有找到 {directory}/map.pcd，请先保存地图。')
    floor = floor_height(directory) if floor is None else floor
    if not math.isfinite(floor):
        raise ValueError('地面高度必须是有限的米数。')
    target = directory / 'nav2'
    store = directory / '.dogroute-grids'
    digest = pcd_digest(directory)
    if target.exists() or target.is_symlink():
        if not target.is_symlink() or target.resolve().parent != store.resolve():
            raise ValueError(f'{target} 已由其它程序使用，保留现有文件。')
        meta = json.loads((target / 'projection.json').read_text())
        if (meta.get('pcd_sha256') == digest and meta['floor_z'] == floor and
                all((target / f).is_file() for f in ('map.pgm', 'map.yaml'))):
            print(f'栅格已就绪：{target}/map.yaml，地面 z={floor:g} m')
            return
        if running(DRAW) and unit_map(DRAW) == directory.name:
            raise ValueError('关闭当前地图的循线服务后再更新地面高度：dogroute stop')
    store.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='grid-', dir=store))
    try:
        subprocess.run(['/usr/bin/python3', str(TOOLS / 'navigation_preview/build_grid.py'),
                        str(directory), '--output', str(output), '--floor-z', str(floor),
                        '--min-height', '0.30', '--max-height', '0.90', '--min-points', '3'], check=True)
        record = output / 'projection.json'
        meta = json.loads(record.read_text())
        meta['pcd_sha256'] = digest
        record.write_text(json.dumps(meta, indent=2) + '\n')
        link = output / 'publish'
        link.symlink_to(output.relative_to(directory))
        os.replace(link, target)
    except BaseException:
        if target.resolve() != output:
            shutil.rmtree(output, ignore_errors=True)
        raise
    print(f'{target}/map.yaml，地面 z={floor:g} m，分辨率 5 cm。')


def locate(directory):
    from interface.srv import Relocalize, IsValid
    pcd = directory / 'map.pcd'
    if not pcd.is_file():
        raise ValueError(f'没有找到 {pcd}')
    stop(DRAW)
    if running(LOCALIZER) and unit_map(LOCALIZER) != directory.name:
        stop(LOCALIZER)
    if not running(LOCALIZER):
        check_no_external_slam()
        launch(LOCALIZER, directory.name, 'ws_slam', 'localizer', 'localizer_launch.py')
    response = call('/localizer/relocalize', Relocalize, Relocalize.Request(pcd_path=str(pcd)))
    if not response.success:
        raise ValueError(response.message)
    deadline = time.monotonic() + 35
    while time.monotonic() < deadline:
        if call('/localizer/relocalize_check', IsValid, IsValid.Request(code=0)).valid:
            print(f'{directory.name} 定位已匹配。请在 RViz 核对固定物体和机身朝向。')
            return
        time.sleep(.4)
    raise ValueError('定位还在等待匹配。核对是否回到了本图起点、朝向是否一致，再执行 dogroute locate。')


def check_localizer(name):
    if not running(LOCALIZER) or unit_map(LOCALIZER) != name:
        raise ValueError(f'请先在 {name} 的建图起点执行 dogroute locate。')


def draw(directory):
    check_localizer(directory.name)
    grid(directory, None)
    if running(DRAW) and unit_map(DRAW) == directory.name:
        if subprocess.run(['pgrep', '-f', r'^\S*/rviz2 .*__node:=office4_draw'], stdout=subprocess.DEVNULL).returncode == 0:
            print(f'{directory.name} 画线窗口已打开。')
            return
    stop(DRAW)
    launch(DRAW, directory.name, 'ws_route', 'office4_route', 'office4.launch.py')
    from lifecycle_msgs.srv import GetState
    deadline = time.monotonic() + 35
    for node in ('map_server', 'controller_server'):
        while call(f'/line_route/{node}/get_state', GetState, GetState.Request()).current_state.label != 'active':
            if time.monotonic() > deadline:
                raise ValueError(f'{node} 暂未就绪，请检查定位和雷达。')
            time.sleep(.4)
    print(f'{directory.name} 栅格和控制器已就绪。点 Draw Route，从机身中心拖到终点，松手保存。')


def main():
    parser = argparse.ArgumentParser(description='建图、定位、画线和循线；地图名仅在 use 时填写。')
    subs = parser.add_subparsers(dest='cmd', required=True)
    subs.add_parser('use', help='选择地图（跨终端记住）').add_argument('name')
    subs.add_parser('env', help='打印当前地图的环境变量（可 eval 使用）')
    subs.add_parser('grid', help='生成二维栅格').add_argument('--floor-z', type=float, metavar='METRES')
    subs.add_parser('walk', help='执行已保存路线').add_argument('--resume', action='store_true')
    for action, text in [('mapping','开始新建图'), ('save','保存并结束本次建图'), ('locate','在原起点定位'),
                         ('draw','打开栅格和画线界面'), ('stand','取权并进入零速 Walking'),
                         ('release','交还 SDK 控制权'), ('stop','关闭画线和定位'), ('status','查看选择和运行状态')]:
        subs.add_parser(action, help=text)
    args = parser.parse_args()
    STATE.mkdir(parents=True, exist_ok=True)
    name = saved_name()
    if args.cmd == 'status':
        print('所选地图：', name or '尚未选择；执行 dogroute use 地图名')
        if name:
            directory = MAPS / name
            marks = [(tag, (directory / rel).is_file()) for tag, rel in
                     (('map.pcd', 'map.pcd'), ('nav2/map.yaml', 'nav2/map.yaml'),
                      ('routes/hand_drawn.json', 'routes/hand_drawn.json'))]
            print('地图目录：', directory)
            print('文件情况：', '  '.join(f'{tag}={"有" if ok else "缺"}' for tag, ok in marks))
        for unit in (MAPPER, LOCALIZER, DRAW):
            if running(unit):
                print(f'{unit:<20} 运行中，地图={unit_map(unit)}' +
                      ('' if service_map(unit) else '（旧入口启动，未记录地图）'))
            else:
                print(f'{unit:<20} 已停止')
        return
    if args.cmd == 'env':
        # 供手工终端使用：eval "$(dogroute env)"
        print(shell_exports(name))
        return
    with (STATE / 'command.lock').open('a') as command_lock, open('/tmp/office4-follow.lock','a') as follow_lock:
        fcntl.flock(command_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Re-read under the lock so a concurrent selection cannot change this command's map.
        name = saved_name()
        if args.cmd != 'walk':
            fcntl.flock(follow_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.cmd == 'use':
            name = map_name(args.name)
            if running(MAPPER) and service_map(MAPPER) != name:
                raise ValueError('先用 dogroute save 保存当前建图，再切地图。')
            for unit in (DRAW, LOCALIZER):
                if running(unit) and unit_map(unit) != name:
                    stop(unit)
            temp = SELECTED.with_suffix('.tmp')
            temp.write_text(name + '\n')
            os.replace(temp, SELECTED)
            print(f'已选择 {name}：{MAPS / name}')
            if not (MAPS / name).is_dir():
                print('这张图还没有目录，新建请接着执行 dogroute mapping。')
            return
        if args.cmd == 'stop':
            stop(DRAW)
            stop(LOCALIZER)
            print('画线和定位已关闭。正在采集的建图请用 dogroute save 保存。')
            return
        if args.cmd in ('stand', 'release'):
            subprocess.run(['bash', str(TOOLS / 'office4_route' / (args.cmd + '.bash'))], check=True)
            return
        if not name:
            raise ValueError('先执行 dogroute use 地图名。')
        directory = MAPS / name
        if running(MAPPER) and args.cmd not in ('mapping', 'save'):
            raise ValueError('建图正在运行；先执行 dogroute save。')
        if args.cmd == 'walk':
            check_localizer(name)
            if not running(DRAW) or unit_map(DRAW) != name:
                raise ValueError(f'请先运行 dogroute draw，为 {name} 打开画线界面并保存路线。')
            os.environ['DOG_ROUTE_MAP'] = name
            os.set_inheritable(command_lock.fileno(), True)
            os.execv('/bin/bash', ['bash', str(SCRIPTS / 'walk.bash')] + (['--resume'] if args.resume else []))
        if args.cmd == 'grid':
            grid(directory, args.floor_z)
        else:
            {'mapping': mapping, 'save': save, 'locate': locate, 'draw': draw}[args.cmd](directory)


if __name__ == '__main__':
    try:
        main()
    except BlockingIOError:
        sys.exit('另一项地图操作或循线正在执行，请在它结束后再操作。')
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
