#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断 PCD z 分布, 找真实地面"""
import sys

import numpy as np

path = sys.argv[1]
data = open(path, 'rb').read()
idx = data.index(b'DATA binary\n') + len(b'DATA binary\n')
pts = np.frombuffer(data[idx:], dtype=np.float32).reshape(-1, 4)
x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
print(f'N={len(pts)}  x[{x.min():.2f},{x.max():.2f}]  y[{y.min():.2f},{y.max():.2f}]  z[{z.min():.2f},{z.max():.2f}]')

hist, edges = np.histogram(z, bins=160)
top = np.argsort(hist)[::-1][:10]
print('--- z 直方图 top-10 峰 ---')
for i in sorted(top, key=lambda k: edges[k]):
    print(f'z [{edges[i]:7.3f},{edges[i+1]:7.3f}]  n={hist[i]:6d}')

print('--- 下半区累积 ---')
for zz in [-0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0]:
    print(f'z < {zz:+.1f}: {(z < zz).sum()}')

# 低 z 区间内的细峰 (bins=40 聚焦 z<0.2)
low = z[z < 0.2]
if len(low):
    h2, e2 = np.histogram(low, bins=40)
    for i in np.argsort(h2)[::-1][:5]:
        print(f'low-z 峰 [{e2[i]:7.3f},{e2[i+1]:7.3f}] n={h2[i]}')
