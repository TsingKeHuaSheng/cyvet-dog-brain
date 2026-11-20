#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PCD / localizer map_cloud -> 2D 占用栅格 (pgm + yaml, Nav2 map_server 可用)

用法:
  python3 map_pgm_gen.py --pcd ~/robot_ws/maps/run3/map.pcd \
      --out-dir ~/robot_ws/maps/run3/grid [--z-lo -0.1 --z-hi 0.6]
  # 无 --pcd 时订阅 /localizer/map_cloud (降采样后仅 ~13k 点, 墙体有空洞, 不推荐)

障碍切片语义: --z-lo/--z-hi 为绝对 z (map 系, z=0=建图起始雷达平面)。Mid-360
垂直 FOV -7°~52°, 雷达打不到近处地面, 室内地判断不可靠, 推荐绝对切片
[-0.1, 0.6] 提取墙体等垂直结构并剔除天花板 (z≈1.83)。不给时回退相对地面
(z 直方图 argmax, 室内常误锁天花板)。
"""
import argparse
import os
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    _BaseNode = Node
except ImportError:          # 离线 PCD 模式无需 ROS
    _BaseNode = object


def load_pcd_xyz(path):
    """解析二进制 PCD (FIELDS x y z [intensity]...), 返回 (N,3) float64"""
    data = open(path, 'rb').read()
    hdr = data[:400].split(b'DATA binary\n')[0].decode(errors='ignore')
    fields = next(l.split()[1:] for l in hdr.splitlines() if l.startswith('FIELDS'))
    n = int(next(l.split()[1] for l in hdr.splitlines() if l.startswith('POINTS')))
    idx = data.index(b'DATA binary\n') + len(b'DATA binary\n')
    arr = np.frombuffer(data[idx:idx + n * 4 * len(fields)], dtype=np.float32)
    arr = arr.reshape(n, len(fields))
    return arr[:, [fields.index('x'), fields.index('y'), fields.index('z')]].astype(np.float64)


class _Log:
    """PCD 模式的极简 logger (generate_map 的 log 参数)"""
    @staticmethod
    def info(msg):
        print(f'[INFO] {msg}')

    @staticmethod
    def warn(msg):
        print(f'[WARN] {msg}')


def generate_map(pts, a, log):
    z = pts[:, 2]
    if a.z_lo is not None and a.z_hi is not None:
        lo, hi = a.z_lo, a.z_hi
        mode = '绝对 z 切片'
    else:
        # 回退: 相对地面 (z 直方图最高峰; 室内场景峰常在天花板, 不可靠)
        hist, edges = np.histogram(z, bins=120)
        ground_z = (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1]) / 2.0
        lo, hi = ground_z + a.slice_lo, ground_z + a.slice_hi
        mode = f'相对地面 z={ground_z:.3f}'
    obs = pts[(z >= lo) & (z <= hi)]
    log.info(f'{len(pts)} 点, {mode}, 障碍切片 [{lo:.2f},{hi:.2f}], 障碍点 {len(obs)}')

    res = a.resolution
    margin = a.margin
    min_x, min_y = obs[:, 0].min() - margin, obs[:, 1].min() - margin
    max_x, max_y = obs[:, 0].max() + margin, obs[:, 1].max() + margin
    w = int(np.ceil((max_x - min_x) / res))
    h = int(np.ceil((max_y - min_y) / res))
    log.info(f'栅格 {w}x{h} @ {res}m, origin=({min_x:.2f},{min_y:.2f})')

    # 0=free(白), 205=unknown(灰), 254=occupied(黑)
    grid = np.full((h, w), 205, dtype=np.uint8)
    # 全部点投影过的格子算 free (走过/看过, 含地面/天花板反射)
    ix = np.clip(((pts[:, 0] - min_x) / res).astype(int), 0, w - 1)
    iy = np.clip(((pts[:, 1] - min_y) / res).astype(int), 0, h - 1)
    grid[iy, ix] = 0
    # 障碍切片覆盖为占据
    ix = np.clip(((obs[:, 0] - min_x) / res).astype(int), 0, w - 1)
    iy = np.clip(((obs[:, 1] - min_y) / res).astype(int), 0, h - 1)
    grid[iy, ix] = 254

    # 孤立障碍去噪: 3x3 邻域内障碍数 <= 2 的散斑转 unknown (纯 numpy)
    occ = (grid == 254).astype(np.int16)
    n = np.zeros_like(occ)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            n += np.roll(np.roll(occ, dy, axis=0), dx, axis=1)
    lonely = (occ == 1) & (n <= 2)
    grid[lonely] = 205
    log.info(f'去噪: 清除孤立障碍格 {int(lonely.sum())}')

    # pgm 行序: 第 0 行 = 世界 y 最大 (map_server 原点在左下)
    grid = np.flipud(grid)
    os.makedirs(a.out_dir, exist_ok=True)
    pgm_path = os.path.join(a.out_dir, 'map.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(grid.tobytes())
    yaml_path = os.path.join(a.out_dir, 'map.yaml')
    with open(yaml_path, 'w') as f:
        f.write('image: map.pgm\n')
        f.write(f'resolution: {res}\n')
        f.write(f'origin: [{min_x:.4f}, {min_y:.4f}, 0.0]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    log.info(f'已生成: {pgm_path} / {yaml_path}')


class MapGen(_BaseNode):
    def __init__(self, args):
        super().__init__('map_pgm_gen')
        self.args = args
        self.done = False
        self.sub = self.create_subscription(
            PointCloud2, args.topic, self.cb, 10)
        self.get_logger().info(f'等待 {args.topic} ...')

    def cb(self, msg):
        if self.done:
            return
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) < 500:
            self.get_logger().warn(f'点数太少 ({len(pts)}), 继续等')
            return
        self.done = True
        generate_map(pts, self.args, self.get_logger())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', required=True)
    p.add_argument('--pcd', default=None,
                   help='直接离线解析 PCD (推荐, 点密度高); 不给则订阅 topic')
    p.add_argument('--resolution', type=float, default=0.05)
    p.add_argument('--margin', type=float, default=1.0)
    p.add_argument('--topic', default='/localizer/map_cloud')
    p.add_argument('--z-lo', type=float, default=None,
                   help='障碍切片绝对 z 下沿 (map 系, 推荐 -0.1)')
    p.add_argument('--z-hi', type=float, default=None,
                   help='障碍切片绝对 z 上沿 (map 系, 推荐 0.6)')
    p.add_argument('--slice-lo', type=float, default=0.05,
                   help='回退: 障碍切片下沿 (相对地面, m)')
    p.add_argument('--slice-hi', type=float, default=0.40,
                   help='回退: 障碍切片上沿 (相对地面, m)')
    args = p.parse_args()
    if args.pcd:
        pts = load_pcd_xyz(args.pcd)
        generate_map(pts, args, _Log)
        return
    rclpy.init()
    node = MapGen(args)
    deadline = time.time() + 60
    try:
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if not node.done:
            print('超时: 未收到 map_cloud. 确认 localizer 已 relocalize 且有订阅者触发发布.')
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
