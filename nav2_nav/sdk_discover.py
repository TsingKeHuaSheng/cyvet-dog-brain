#!/usr/bin/env python3
"""SDK 设备发现探测: 确认狗在不在线 (只发现, 不连接, 不发指令)

用法: python3 sdk_discover.py [iface]   # 默认 wlP1p1s0
"""
import sys
import threading

import robot_motion_sdk as sdk

iface = sys.argv[1] if len(sys.argv) > 1 else 'wlP1p1s0'

devices = {}
cv = threading.Condition()


def cb(sn, info_json):
    with cv:
        devices[sn] = info_json
        cv.notify_all()


sdk.service.set_network_interface(iface)
sdk.service.set_discover_callback(cb)
if not sdk.service.initial(None, 'nav-probe'):
    print('[FAIL] sdk.service.initial')
    sys.exit(2)
try:
    for attempt in range(2):
        with cv:
            n_before = len(devices)
        if not sdk.service.discover_devices(timeout_ms=5000):
            print('[FAIL] discover_devices call')
            sys.exit(2)
        with cv:
            if len(devices) > n_before:
                break
        print('[WARN] 5s 无发现回调, 重试一次')
    print(f'发现 {len(devices)} 台设备 (iface={iface}):')
    for sn, info in devices.items():
        print(f'  SN={sn}  info={str(info)[:150]}')
    sys.exit(0 if devices else 1)
finally:
    sdk.service.shutdown()
