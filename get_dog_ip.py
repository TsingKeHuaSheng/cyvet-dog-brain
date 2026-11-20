#!/usr/bin/env python3
"""通过 UniUbi SDK 发现机器狗，打印其当前 Wi-Fi IP（不依赖 IP 是否变化）。

用法:  python3 ~/get_dog_ip.py [--iface wlP1p1s0] [--sn E03C1CBAAC40A101]
输出:  纯 IP（便于 shell 捕获），详细信息走 stderr
"""
import argparse, json, re, subprocess, sys, os

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default=os.environ.get("UNIUBI_IFACE", "wlP1p1s0"))
    ap.add_argument("--sn", default=os.environ.get("UNIUBI_DOG_SN", ""))
    ap.add_argument("--timeout", default="12")
    a = ap.parse_args()

    ex = os.path.expanduser("~/uniubi_robot_sdk_py/examples/example_highlevel.py")
    py = os.path.expanduser("~/venvs/uniubi/bin/python")
    cmd = [py, ex, "--iface", a.iface, "--discover-only", "--discovery-timeout", a.timeout]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=90, cwd="/tmp")
    except subprocess.TimeoutExpired:
        print("发现超时", file=sys.stderr); return 2
    out = r.stdout + r.stderr

    # 匹配 "SN=xxx info={...}" 里的 wlan.ipv4Addr
    for line in out.splitlines():
        m = re.search(r"SN=([0-9A-Za-z]+)\s+info=(\{.*\})\s*$", line)
        if not m:
            continue
        sn, raw = m.group(1), m.group(2)
        if a.sn and sn != a.sn:
            continue
        try:
            info = json.loads(raw)
        except json.JSONDecodeError:
            continue
        net = info.get("network", {})
        ip = (net.get("wlan") or {}).get("ipv4Addr", "")
        eth = (net.get("ether") or {}).get("ipv4Addr", "")
        ver = info.get("version", "?")
        print(f"SN={sn} model={info.get('deviceModel')} fw={ver} wlan={ip or '-'} eth={eth or '-'}",
              file=sys.stderr)
        if ip:
            print(ip)
            return 0
    print("未发现匹配的机器狗", file=sys.stderr)
    return 1

if __name__ == "__main__":
    sys.exit(main())
