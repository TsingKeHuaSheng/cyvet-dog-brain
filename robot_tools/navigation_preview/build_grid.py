#!/usr/bin/env python3
"""Build a flat-ground planning preview from PGO body patches and global poses.

Height limits are expressed in the saved map frame. This is not a terrain or
negative-obstacle detector. Unknown cells remain unknown; original PCDs are read-only.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def read_pcd(path):
    with Path(path).open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f'Missing DATA header: {path}')
            fields = line.decode('ascii').strip().split()
            if not fields or fields[0].startswith('#'):
                continue
            header[fields[0]] = fields[1:]
            if fields[0] == 'DATA':
                break
        if header['DATA'] != ['binary']:
            raise ValueError('Only uncompressed binary PGO PCDs are supported')
        counts = header.get('COUNT', ['1'] * len(header['FIELDS']))
        dtype = []
        for name, kind, size, count in zip(header['FIELDS'], header['TYPE'], header['SIZE'], counts):
            scalar = '<' + {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + size
            dtype.append((name, scalar) if int(count) == 1 else (name, scalar, (int(count),)))
        data = np.frombuffer(stream.read(), dtype=np.dtype(dtype))
    xyz = np.column_stack([data[name] for name in ('x', 'y', 'z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


def rotation(q):
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def ray_cells(start, end):
    x, y = start
    ex, ey = end
    dx, dy = abs(ex-x), -abs(ey-y)
    sx, sy = (1 if x < ex else -1), (1 if y < ey else -1)
    err = dx + dy
    while True:
        yield x, y
        if x == ex and y == ey:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map_dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--floor-z', type=float, required=True)
    parser.add_argument('--min-height', type=float, default=0.05)
    parser.add_argument('--max-height', type=float, default=0.85)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--range', dest='max_range', type=float, default=8.0)
    parser.add_argument('--min-points', dest='min_points', type=int, default=3,
                        help='Minimum endpoint hits for a cell to count as obstacle (noise filter)')
    args = parser.parse_args()
    if args.resolution <= 0 or not 0 <= args.min_height < args.max_height:
        parser.error('Invalid resolution or height range')
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'map.yaml').exists():
        parser.error('Output already exists; choose a new directory')
    cloud = read_pcd(args.map_dir / 'map.pcd')
    origin = np.floor((cloud[:, :2].min(axis=0) - 0.5) / args.resolution) * args.resolution
    shape = np.ceil((cloud[:, :2].max(axis=0) + 0.5 - origin) / args.resolution).astype(int) + 1
    if shape.prod() > 20_000_000:
        raise ValueError('Map exceeds 20 million cells')
    width, height = map(int, shape)
    free = np.zeros((height, width), dtype=bool)
    occupied = np.zeros_like(free)
    hits = np.zeros((height, width), dtype=np.int32)
    scan_cache = []
    scans = 0
    retained = 0
    for line in (args.map_dir / 'poses.txt').read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError('Expected patch name, xyz and quaternion wxyz')
        trans = np.array(parts[1:4], dtype=float)
        points = read_pcd(args.map_dir / 'patches' / parts[0]) @ rotation(list(map(float, parts[4:8]))).T + trans
        distance = np.linalg.norm(points[:, :2] - trans[:2], axis=1)
        select = ((points[:, 2] >= args.floor_z + args.min_height) &
                  (points[:, 2] <= args.floor_z + args.max_height) &
                  (distance >= 0.10) & (distance <= args.max_range))
        points = points[select]
        distance = distance[select]
        if not len(points):
            continue
        # Keep nearest height-band obstacle for each angular bin to avoid
        # clearing behind an obstacle from an aggregated scan.
        angles = np.arctan2(points[:, 1]-trans[1], points[:, 0]-trans[0])
        bins = np.floor((angles + np.pi) * 720 / (2*np.pi)).astype(int)
        order = np.argsort(distance)
        _, unique = np.unique(bins[order], return_index=True)
        endpoints = points[order[unique]]
        cells = np.floor((endpoints[:, :2] - origin) / args.resolution).astype(int)
        start = np.floor((trans[:2] - origin) / args.resolution).astype(int)
        scan_cache.append((start, cells))
        for ex, ey in cells:
            if 0 <= ex < width and 0 <= ey < height:
                hits[ey, ex] += 1
        scans += 1
        retained += len(endpoints)
    if scans == 0:
        raise ValueError('No scans in height band; check floor height and source map')
    # 噪声过滤：只有被命中 >= min_points 次的格子才算障碍
    occupied = hits >= max(1, args.min_points)
    for start, cells in scan_cache:
        for ex, ey in cells:
            if not (0 <= ex < width and 0 <= ey < height):
                continue
            # 不论该格是否通过噪声过滤，射线都要投出去（标出可走区域）
            for x, y in ray_cells(tuple(start), (int(ex), int(ey))):
                if 0 <= x < width and 0 <= y < height:
                    free[y, x] = True
    pixels = np.full((height, width), 205, dtype=np.uint8)
    pixels[free] = 254
    pixels[occupied] = 0  # A later ray never erases an observed obstacle.
    with (args.output / 'map.pgm').open('wb') as out:
        out.write(f'P5\n{width} {height}\n255\n'.encode())
        out.write(np.flipud(pixels).tobytes())
    (args.output / 'map.yaml').write_text(
        f'image: map.pgm\nresolution: {args.resolution}\n'
        f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    summary = dict(source=str(args.map_dir.resolve()), floor_z=args.floor_z,
                   obstacle_z=[args.floor_z+args.min_height, args.floor_z+args.max_height],
                   resolution=args.resolution, size=[width, height], patches=scans,
                   occupied_cells=int(occupied.sum()), free_cells=int((free & ~occupied).sum()),
                   unknown_cells=int((~free & ~occupied).sum()), retained_rays=retained,
                   min_points_per_cell=args.min_points,
                   raw_obstacle_hits=int((hits > 0).sum()),
                   limitations='Approximate flat-ground height-band projection; no drop-off detection or traversability guarantee. Validate in RViz; not validated for motion.')
    (args.output / 'projection.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
