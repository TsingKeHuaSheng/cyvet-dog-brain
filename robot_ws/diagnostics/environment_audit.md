# 环境审计（只读实测）

采集时间：2026-11-12 11:47 ~ 12:52 (Asia/Shanghai)
采集方式：由 Mac 通过 `ssh nvidia@192.168.100.191` 远程执行

## 1. 硬件 / 系统

| 项目 | 实测值 |
|---|---|
| 主机名 | `nvidia-desktop` |
| 用户 | `nvidia` (uid 1000, 在 sudo 组) |
| 架构 | `aarch64` |
| 内核 | `5.15.148-tegra` |
| OS | Ubuntu 22.04.5 LTS |
| L4T | R36.4.3（BOARD: generic, 2025-01-08） |
| 载板型号 | **`3767-301-0003-H.1-1-1-jetson-orin-nano-devkit-super`** |
| device-tree model | `NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super` |
| 模块 / 载板 | 模块 `p3767-0003`（Orin Nano 8GB）+ 载板 `p3768-0000`（官方 Devkit） |
| 存储 | NVMe `nvme0n1`，`/` 116G，已用 27G，**可用 83G** |
| 内存 | 7.4 GiB，可用约 5.1 GiB |
| Swap | 4 × zram，共 3.7 GiB |

> 结论：**载板已核实为 NVIDIA 官方 Orin Nano Devkit Super 载板**，不是第三方定制底板。
> 交接文档里"实际载板型号仍需核实"一项，**已关闭**。

## 2. 工具链

| 工具 | 路径 / 版本 |
|---|---|
| git | `/usr/bin/git` 2.34.1 |
| cmake | `/usr/bin/cmake` 3.22.1 |
| g++ | 11.4.0 (Ubuntu 11.4.0-1ubuntu1~22.04.2) |
| make | `/usr/bin/make` |
| python3 | 3.10.12 |
| colcon | **`/usr/bin/colcon` 存在**，colcon-core 0.20.1，`python3-colcon-common-extensions` 已装 |
| ros2 | 未 source 时**不在 PATH**（正常现象，需 `source /opt/ros/humble/setup.bash`） |
| tcpdump | 11:xx 前**未安装**，本次补装 4.99.1（仅 1 个包 497KB） |
| tmux / screen | **均未安装**（验收改用 `setsid nohup` 保活） |

### 关于 `colcon version-check` 卡死（已定论）

- `colcon` **本来就是装好的**，不是缺包。
- `version-check` 的语义是"把本地包版本和 **PyPI** 比对"，会联网；GitHub/PyPI 方向不稳时它会**长时间无输出**。
- 现场发现残留进程：`PID 8697 /usr/bin/python3 /usr/bin/colcon version-check`，已运行 12 分 34 秒（STAT `S+`）。
- 处理：`kill -INT 8697` 即退出。**未使用 pkill 批量杀 Python 进程。**
- 以后判断 colcon 是否存在，请用 `command -v colcon` / `colcon --help`，**不要再用 `version-check`**。

## 3. 网络

### 现状（验收通过时）

```
wlP1p1s0   UP   192.168.100.191/24        # Wi-Fi，SSID 清科华胜
enP8p1s0   UP   192.168.1.50/24           # 有线，直连 MID-360
default via 192.168.100.1 dev wlP1p1s0 proto dhcp metric 600
192.168.1.0/24 dev enP8p1s0 proto kernel scope link src 192.168.1.50 metric 700
```

- 有线口 MAC：`3c:6d:66:c3:ad:c8`，**Speed 100Mb/s / Full duplex / Link detected: yes**
- 访问雷达的路由：`192.168.1.194 dev enP8p1s0 src 192.168.1.50`（正确）
- ARP：`192.168.1.194 lladdr 34:91:f0:e4:0d:c8`
- Ping：4/4，0% 丢包，RTT 0.17 ~ 3.3 ms
- 防火墙：`iptables` INPUT 策略 ACCEPT 且**无任何规则**；`ufw` 未安装；`rp_filter=2`（宽松）
- DNS：`127.0.0.53`（systemd-resolved），未被改动

### 「192.168.1.50 会消失」的根因（已定位并修复）

原配置位于：

```
/run/NetworkManager/system-connections/enP8p1s0.nmconnection
```

`/run` 是 **tmpfs（内存盘）**，**每次重启必然清空**；且该 profile `connection.autoconnect = no`。
两个因素叠加 = 开机后地址必然丢失，必须手工 `ip addr add`。

另有一个自动生成的 `Wired connection 1`（`ipv4.method=auto`，DHCP，autoconnect=yes）在 `/run` 里，
若它先匹配到 `enP8p1s0`，会在雷达专线上跑 DHCP，属潜在干扰项。

**处理**：新建持久化 profile `mid360-direct`（见 `mid360_config_summary.md`），写入 `/etc`。
现在只保留 3 个 profile：`清科华胜`(Wi-Fi, /etc)、`mid360-direct`(有线, /etc)、`Wired connection 1`(/run, 自动生成，无接口绑定)。

> ⚠️ **重启持久性未经实测**（本次未重启 Jetson）。已验证的是：配置文件位于 `/etc` 且 `autoconnect=yes`。

## 4. ROS 2

| 项目 | 值 |
|---|---|
| 发行版 | Humble，`/opt/ros/humble` |
| 包数量 | 261 |
| `ROS_DISTRO` | 交互式 shell 中已设置；非交互需手动 source |
| 中间件 | 默认 RMW（FastDDS），`ROS_DOMAIN_ID` 未设置（=0） |

## 5. 审计前的 Livox 相关状态

- `ros2 pkg list | grep -i livox` → **无**
- `ldconfig -p | grep -i livox` → **无**
- `/usr/local/include/livox*`、`/usr/local/lib/liblivox*` → **无**
- dpkg 无 livox 包
- 唯一已有的相关物：`/home/nvidia/Viewer2_2.6.2_Linux/`（Livox Viewer 2 图形工具，未使用）

> 即：审计时 **SDK2 与 ROS 驱动均未安装**，本次为全新安装。

## 6. 其他已存在、本次未触碰的内容

`~/JEP`、`~/jetson-inference`、`~/ultralytics`、torch/torchvision/onnxruntime 的 whl 包、`~/Viewer2_2.6.2_Linux`。
**本次未升级、未重装、未删除上述任何内容。**
