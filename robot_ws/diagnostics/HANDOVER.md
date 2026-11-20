# 机器狗二次开发 · 项目交接报告

生成时间：2026-11-13 12:50（Asia/Shanghai）
生成方式：全部由 SSH 登录 Jetson 实测采集，命令与输出见各节
交接人视角：本报告只记录**可验证的事实**，推测与未验证内容单独标注

## 事实分级约定

| 标记 | 含义 |
|---|---|
| 【已验证】 | 本次或前次在机器上实际执行过，有命令输出/文件/抓包为证 |
| 【文档推测】 | 来自官方 README/文档，未在本次环境复现 |
| 【未验证】 | 完全没有执行过，不能假定可用 |
| 【待确认】 | 需要现场操作或追加命令才能确定 |

---

# 一、项目背景

| 项目 | 内容 |
|---|---|
| 机器人 | UniUbi 灵猫 Cyvet 四足机器人（型号 CYVET-CREATOR） |
| 雷达 | Livox MID-360S（注意：**不是** MID-360） |
| 计算板 | NVIDIA Jetson Orin Nano 8GB（外部板，**非**宇泛大脑板） |
| 系统 | Ubuntu 22.04 + ROS 2 Humble |
| 最终目标 | 建图 → 定位 → 规划 → 避障 → 从 A 点运动到 B 点 |
| 控制方式 | UniUbi HighLevel SDK，机器狗经 Wi-Fi 连接 Jetson |
| 雷达连接 | MID-360S 经以太网线直连 Jetson |
| 开发方式 | Mac 通过 SSH 登录 Jetson |

---

# 二、已完成工作逐项核对

## 2.1 系统与工具链

| # | 项目 | 状态 | 实际证据 |
|---|---|---|---|
| 1 | Jetson 可 SSH | 【已验证】 | `ssh -o BatchMode=yes nvidia@192.168.100.191` 返回 0，可执行命令 |
| 2 | 用户名 / 登录方式 | 【已验证】 | 用户 `nvidia`；**已配置公钥免密登录**（见 4.3 备份说明）；现场提供密码为 `nvidia` |
| 3 | IP / 网卡 | 【已验证】 | `wlP1p1s0` = `192.168.100.191/24`；`enP8p1s0` = `192.168.1.50/24` |
| 4 | JetPack / L4T | 【已验证】 | `R36 (release), REVISION: 4.3` |
| 5 | Ubuntu | 【已验证】 | `Ubuntu 22.04.5 LTS` |
| 6 | CPU 架构 | 【已验证】 | `aarch64`，kernel `5.15.148-tegra` |
| 7 | ROS 2 | 【已验证】 | `ROS_DISTRO=humble`，基础包 261 个 |
| 8 | Python | 【已验证】 | `Python 3.10.12` |
| 9 | C++ 编译环境 | 【已验证】 | `g++ 11.4.0`、`cmake 3.22.1`、`git 2.34.1` |

补充硬件信息【已验证】：

```
model : NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
TNSPEC: 3767-301-0003-H.1-1-1-jetson-orin-nano-devkit-super-
```

> 结论：**Orin 是 NVIDIA 官方 Devkit**，不是宇泛定制大脑板。这是后续"为什么用 aarch64_host 而不是 aarch64"的根本原因。

## 2.2 UniUbi SDK

| # | 项目 | 状态 | 实际证据 |
|---|---|---|---|
| 10 | Python SDK 安装 | 【已验证】 | `pip show uniubi-robot-motion-sdk` → `Name: uniubi-robot-motion-sdk` / `Version: 1.0.0`；`import robot_motion_sdk` 成功，`MEDIA_ENABLED = True` |
| 11 | C++ SDK | 【已验证，但有前提】 | 预编译运行库存在，**未安装到系统目录**，靠 `LD_LIBRARY_PATH` 引用（见下方说明） |
| 12 | SDK 对应固件版本 | 【已验证】 | 两个仓库 tag 均为 `Cyvet-V1.00.000`；机器狗 `system.version` 上报 `Cyvet-V1.00.000`，三者一致 |
| 13 | Python 虚拟环境 | 【已验证】 | `/home/nvidia/venvs/uniubi` |
| 14 | SDK 源码目录 | 【已验证】 | C++：`/home/nvidia/uniubi_robot_sdk`（68M）；Python：`/home/nvidia/uniubi_robot_sdk_py`（7.8M） |
| 15 | SDK 依赖目录 | 【已验证】 | `/home/nvidia/uniubi_robot_sdk/lib/aarch64_host/`，共 **28 个文件** |

### 关于第 11 项的重要说明

`lib/aarch64_host/` 里的库**没有**执行 `make install`，也没有放进 `/usr/lib`。运行时通过环境变量指向该目录：

```bash
export UNIUBI_SDK_ROOT="$HOME/uniubi_robot_sdk"
export LD_LIBRARY_PATH="$UNIUBI_SDK_ROOT/lib/aarch64_host${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

不设置会直接报错（实测）：

```
ImportError: librobotMotionSdk.so: cannot open shared object file: No such file or directory
```

所以**"SDK 是否安装"的准确表述是**：Python 层装进了 venv；C++ 运行库只是解压到位并由环境变量引用，未做系统级安装。删掉 `~/uniubi_robot_sdk` 即等于卸载 C++ 层。

### 关键 .so 清单【已验证】

```
librobotMotionSdk.so   libmediaBus.so   libubase.so   libudbus.so   libuutils.so
+ 自带第三方依赖：libddsc/libddscxx (CycloneDDS 0.10.5)、libiceoryx_*、
  libcrypto/libssl 1.1、libz、libacl、libattr
```

## 2.3 运行过的示例与实际执行到哪一步

| # | 项目 | 状态 | 说明 |
|---|---|---|---|
| 16 | 运行过哪些示例 | 【已验证】 | 仅运行 `examples/example_highlevel.py`。未运行 `example_lowlevel.py`、`example_media_frames.py`、`example_audio*.py` |
| 17 | 是否成功执行过 HighLevel | 【已验证】 | 是。参数 `--iface wlP1p1s0 --device-id E03C1CBAAC40A101 --read-only` |
| 18 | system / state / motors | 【已验证】 | 三项均成功，见下 |
| 18b | sensor / odom | 【未验证】 | **从未执行过**，不知道输出内容 |
| 19 | take / start walking / send / stop / release | 【未验证】 | **本报告作者从未执行**。使用方口述 2026-11-12 曾在 Orin 上跑通"站立、行走"，但**未留下日志证据** |
| 20 | 当前是否可远程控制 | 【待确认】 | 见 2.5，**当前 SDK 发现失败** |

### 第 18 项的实际输出摘要【已验证】

`system`：
```
"battery": { "power": 73.0, "voltage": 46.72999954223633, "remaining": 7.279999732971191 }
"motionOwner": 0,
"motorReady": true,
```
（2026-11-12 首测时为电量 84%，2026-11-13 复测为 73%，说明设备持续待机耗电）

`state`：返回 `{}`，表示当前无进行中动作（官方语义：无活动动作时合法返回空对象）

`motors`：`motorNum=12`，命名如下
```
leftFrontLeg / rightFrontLeg / leftRearLeg / rightRearLeg
  × hip / thigh / calf
```

### 关于第 19 项 —— 必须如实记录

使用方于 2026-11-12 口述："我在 Orin 上跑了一下狗的运动，站立行走正常"。

但本报告作者**没有找到任何对应日志**：

```
~/.ros/log/ 下最近一次记录为 2026-11-12-17-51-54，内容只有：
  [INFO] [launch]: process started with pid [17927]   ← livox_ros_driver2_node 启动
~/.ros/log/python3_*.log 为 0 字节
~/.bash_history 中无 take / start walking / send 等记录
```

**结论：这条属于"使用方口述，未经验证"。接手人应重新验证，不能直接当既有能力使用。**

## 2.4 Jetson 与机器狗的实际连通性（2026-11-13 12:50 实测）

| 检查 | 结果 | 证据 |
|---|---|---|
| ping 机器狗 `192.168.100.114` | **PASS** | 0% 丢包 |
| 机器狗 MAC 核对 | **PASS** | `fc:37:6d:67:ab:5f`，与已知一致 |
| RTSP 554 端口 | **PASS** | TCP 可连 |
| **UniUbi SDK 发现** | **FAIL** | `get_dog_ip.py` 返回"未发现匹配的机器狗" |

### 失败原因已定位到"狗侧无应答"【已验证】

抓包显示 **Jetson 侧正常发出 DDS 发现包**：

```
12:47:05 IP 192.168.100.191.54673 > 239.255.0.1.17900: UDP, length 416
12:47:06 IP 192.168.100.191.54673 > 239.255.0.1.17900: UDP, length 416
12:47:14 IP 192.168.100.191.54673 > 239.255.0.1.17900: UDP, length 416
```

但**没有任何来自 `192.168.100.114` 的应答包**。

对比：2026-11-12 17:44 时同一命令可成功连接。之后 Jetson 于 11-13 11:52 重启，机器狗状态未知。

**判断【待确认】**：机器狗的运控服务当前未响应 DDS 发现。可能原因（按可能性排序，均未验证）：
1. 机器狗运控服务未启动 / 处于特殊状态（例如遥控器已连接）
2. 机器狗被断电重启，服务尚未就绪
3. 其他现场原因

**这不是 Jetson 网络问题**——ping 与 RTSP 都正常。

## 2.5 未安装的组件【已验证】

在 `source /opt/ros/humble/setup.bash` 后逐包检查，以下**全部为 0**：

```
slam_toolbox               0
nav2_bringup               0
pointcloud_to_laserscan    0
robot_localization         0
fast_lio                   0
```

已安装并可见的只有：`livox_ros_driver2`（需 `source ~/ws_livox/install/setup.bash` 后才可见）。

> 即：**建图、定位、导航相关软件一个都没装**。当前只到"传感器可用 + SDK 可用"。

---

# 三、硬件连接与网络拓扑

## 3.1 实测拓扑

```
MacBook (192.168.100.105)
   │  SSH（公钥免密）
   ↓
Jetson Orin Nano (nvidia-desktop, aarch64)
   ├── wlP1p1s0 ── Wi-Fi「清科华胜」── 路由器 192.168.100.1
   │      地址 192.168.100.191/24
   │      默认路由 192.168.100.1（DHCP, metric 600）
   │      └── 机器狗 wlan0 192.168.100.114（DHCP）
   │
   └── enP8p1s0 ── 网线直连
          地址 192.168.1.50/24（静态，NetworkManager 连接名 mid360-direct）
          metric 700，**无网关、无 DNS、never-default**
          └── Livox MID-360S 192.168.1.194
```

## 3.2 路由表【已验证】

```
default via 192.168.100.1 dev wlP1p1s0 proto dhcp metric 600
192.168.1.0/24   dev enP8p1s0 proto kernel scope link src 192.168.1.50 metric 700
192.168.100.0/24 dev wlP1p1s0 proto kernel scope link src 192.168.100.191 metric 600
```

要点：**默认路由始终走 Wi-Fi**，雷达专线不设网关，因此不影响 SSH。

## 3.3 NetworkManager 连接【已验证】

```
NAME             DEVICE     AUTOCONNECT
清科华胜          wlP1p1s0   yes      ← Wi-Fi
mid360-direct    enP8p1s0   yes      ← 雷达专线（本次新建）
```

`mid360-direct` 配置文件位于 `/etc/NetworkManager/system-connections/mid360-direct.nmconnection`，
**已实测通过两次重启**（2026-11-12 13:24、2026-11-13 11:52），重启后 `192.168.1.50` 自动恢复。

> 历史问题记录：原配置位于 `/run/NetworkManager/system-connections/`（tmpfs，重启即失），
> 且 `autoconnect=no`，导致每次开机需手工 `ip addr add`。此问题已修复。

## 3.4 机器狗自带但未使用的接口【已验证】

| 接口 | 信息 | 状态 |
|---|---|---|
| `eth0` | MAC `e0:3c:1c:ba:ac:40`，link up | **未接线** |
| `eth1` (mobile) | `10.112.0.101/29`，网关 `10.112.0.99` | 4G 模块，SIM 已插，未使用 |

## 3.5 链路质量实测【已验证】

| 链路 | 结果 |
|---|---|
| Wi-Fi → 机器狗（15 次 ping） | 0% 丢包，min 2.5 / avg 16.3 / max 57.5 ms，抖动 20.7 ms |
| 有线 → 雷达（4 次 ping） | 0% 丢包，min 1.4 / avg 1.8 / max 1.9 ms，抖动 0.2 ms |

> 备注：Wi-Fi 链路抖动比有线大一个数量级以上。这是现场观测值，未做多时段统计。

---

# 四、已创建的文件与资产

## 4.1 Jetson 上的新建内容

| 路径 | 内容 | 状态 |
|---|---|---|
| `~/uniubi_robot_sdk/` | C++ 运行库（tag Cyvet-V1.00.000，含 lib/aarch64_host/） | 【已验证】68M |
| `~/uniubi_robot_sdk_py/` | Python 绑定源码（tag Cyvet-V1.00.000） | 【已验证】7.8M |
| `~/venvs/uniubi/` | Python 虚拟环境，已装 uniubi-robot-motion-sdk 1.0.0 | 【已验证】 |
| `~/env_uniubi.sh` | SDK 环境变量脚本 | 【已验证】，内容见 §2.2 |
| `~/get_dog_ip.py` | 通过 SDK 发现机器狗并输出当前 IP | 【已验证】可用 |
| `~/dog_rtsp.sh` | 结合上者输出 RTSP 地址 | 【已验证】曾成功抓帧 |
| `~/ws_livox/` | livox_ros_driver2 工作空间 | 【已验证】 |
| `~/robot_ws/Livox-SDK2/` | Livox SDK2 源码与构建产物 | 【已验证】 |
| `~/robot_ws/diagnostics/` | 诊断文档与日志 | 【已验证】 |

## 4.2 雷达相关配置

| 项目 | 值 | 状态 |
|---|---|---|
| 雷达型号判定 | **MID-360S（dev_type = 35）**，不是 MID-360（类型 9） | 【已验证】从发现回包字节解出 |
| 使用的配置模板 | `Mid360s_config.json`（JSON 块名必须严格为 `"Mid360s"`） | 【已验证】 |
| 配置文件路径 | `~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/Mid360s_config.json` | 【已验证】 |
| host_ip / lidar ip | `192.168.1.50` / `192.168.1.194` | 【已验证】 |
| 新增 launch | `msg_MID360s_pc2_launch.py`（xfer_format=0，输出 PointCloud2） | 【已验证】 |
| 原 launch | `msg_MID360s_launch.py`（xfer_format=1，输出 CustomMsg） | 【已验证】，未改动 |

## 4.3 备份文件

| 备份 | 说明 |
|---|---|
| `~/.ssh/authorized_keys.bak.20261112_114651` | 添加公钥前的原始文件 |
| `~/.config/.../MID360_config.json.orig.*` | 驱动配置文件原样备份 |
| `~/robot_ws/diagnostics/backup/nm-20261112_120024/` | 修改 NetworkManager 前的连接配置原文 |

## 4.4 文档

| 文件 | 内容 |
|---|---|
| `~/robot_ws/diagnostics/environment_audit.md` | 系统/ROS/网络/依赖审计 |
| `~/robot_ws/diagnostics/installation.log` | SDK2 与驱动编译安装日志 |
| `~/robot_ws/diagnostics/mid360_config_summary.md` | 雷达 IP/端口/配置/启停命令 |
| `~/robot_ws/diagnostics/acceptance_report.md` | 雷达数据验收（含重启专项） |
| `~/robot_ws/diagnostics/dog_integration_status.md` | 机器狗接入状态 |
| `~/robot_ws/diagnostics/next_steps.md` | 后续条件与已知坑 |

---

# 五、四类事实汇总

## 5.1 【已验证】事实清单

1. Jetson 可通过 SSH 免密登录，主机名 `nvidia-desktop`
2. 系统：Ubuntu 22.04.5、L4T R36.4.3、aarch64、kernel 5.15.148-tegra
3. 板卡为 NVIDIA 官方 Orin Nano Devkit Super（非宇泛大脑板）
4. ROS 2 Humble 可用，基础包 261 个
5. g++ 11.4.0 / cmake 3.22.1 / python 3.10.12
6. 有线专线 `192.168.1.50/24` 由 NetworkManager 持久化，**两次重启后自动恢复**
7. 默认路由始终在 Wi-Fi，SSH 通路不受影响
8. 雷达 `192.168.1.194` 可 ping，MAC `34:91:f0:e4:0d:c8`
9. 雷达实际为 **MID-360S（dev_type=35）**，必须用 `Mid360s` 配置
10. 雷达点云 10 Hz、IMU 200 Hz，RViz 可实时显示（2026-11-12 实测）
11. SDK2 v1.3.1 已编译安装到 `/usr/local`（`ldconfig` 已注册）
12. livox_ros_driver2 1.2.6 已编译到 `~/ws_livox`
13. UniUbi Python SDK 1.0.0 已装入 venv，`import` 成功，`MEDIA_ENABLED=True`
14. SDK 版本与固件版本严格匹配（均 `Cyvet-V1.00.000`）
15. 编译平台为 `aarch64_host`（CMakeCache 实查）
16. MediaBus 在 host 模式下 **视频不可用**：`get_media_layout` / `start_raw_video_frame` / `start_encoded_video_frame` 全部返回 `kNotSupported`；`setup()` 返回 True
17. `lib/aarch64_host/libmediaBus.so` **零 NVIDIA 媒体库依赖**；`lib/aarch64/libmediaBus.so` 依赖 7 个（libnvbufsurface、libnvbuf_fdmap、libcudart、libcuda、libEGL、libnvos、libnvrm）
18. 机器狗可被 SDK 按 SN 发现（2026-11-12 17:44 时），IP 曾为 `192.168.100.114`
19. 只读查询可用：`system` / `state` / `caps` / `motors`
20. 已读到的动作能力包含 `walking`，其参数范围 `lineVelocityX ±1.5 m/s`、`lineVelocityY ±1.0 m/s`、`velocity ±2.5 rad/s`
21. 双目 RTSP 两路可用，H.265 / 1280×720，Jetson 端可硬解（`nvv4l2decoder`）
22. 双路时间差约 0.16–0.18 秒，十分钟内基本不漂（基于两组截图对比）
23. **当前（2026-11-13 12:50）SDK 发现失败**，抓包证明是狗侧无应答
24. 建图/定位/导航相关 ROS 包**一个都没装**

## 5.2 【文档推测】—— 未在本次环境复现

1. MediaBus 的 PCM 音频采集与 RawBack 播放"远端可用"（README 声明，本次只观察到 `audioCapture` socket 建立，未实际采集音频）
2. "远端视频订阅返回 kNotSupported 且只能走 RTSP"——行为已在本次实测确认；但"为什么"层面的完整设计意图来自文档
3. `aarch64`（大脑版）在非大脑板上会因缺少 `/etc/robot/sdk_config.json` 与板内媒体服务而初始化失败——**推断，未实际装错版本验证**
4. README 提到低层（LowLevel）共享内存控制要求在大脑板运行——**未验证 remote 下是否可用**

## 5.3 【完全未验证】

1. 机器狗在 Jetson 上的**任何运动控制**（take / start walking / move / stop / release）
2. 机器狗里程计（`odom`）是否可用、频率、精度
3. `sensor` 观测的实际内容
4. LowLevel 关节级控制在 Jetson 上是否可用
5. 机身行走时的俯仰/横滚幅值（决定 2D 切片可行性）
6. 雷达在真实/模拟轨道环境下的特征丰富度
7. Nav2、slam_toolbox、FAST-LIO 在本机的实际运行表现
8. 长时间运行稳定性（>1 小时）
9. 断电重启后 SDK 能否自动恢复连接

## 5.4 【待确认】—— 需要现场操作或追加命令

| 项目 | 需要做什么 |
|---|---|
| 机器狗 SDK 服务为何无应答 | 到现场看狗的状态；尝试断电重启；确认遥控器是否已断开 |
| 机器狗当前 IP 是否仍为 `.114` | 重新执行 `~/get_dog_ip.py` |
| 运动控制是否真的可用 | 在架空/垫起条件下，由人在场重新验证并保留日志 |
| 机身姿态幅度 | 录 `/livox/imu`，分析行走时 pitch/roll 峰峰值 |
| 里程计质量 | 写 odom bridge 后，比对"指令位移 vs odom 位移" |

---

# 六、还缺少的条件

## 6.1 软件缺失【已验证】

```
建图：无
定位：无
规划/避障：无
运动桥接节点：无（odom bridge、cmd_vel bridge 均未编写）
```

## 6.2 需要新增的自研模块

| 模块 | 作用 | 现状 |
|---|---|---|
| `uniubi_odom_bridge` | SDK odom → `/odom` + TF `odom→base_link` | 未编写 |
| `uniubi_cmd_vel_bridge` | `/cmd_vel` → SDK `move(vx,vy,vyaw)`，含超时停车与速度限幅 | 未编写 |
| bringup | 统一启动 | 未编写 |

## 6.3 现场条件

| 条件 | 用途 |
|---|---|
| 机器狗可断电重启/可操作 | 解决当前 SDK 无应答 |
| 有人能带狗走动 | 采集建图数据、验证里程计 |
| 架空或垫起条件 | 首次运动测试的安全前提 |

---

# 七、接手人建议的第一步

按优先级：

1. **先解决机器狗 SDK 无应答**（阻断项）
   - 现场确认机器狗状态，检查遥控器是否处于断开状态
   - 重启后执行：`source ~/env_uniubi.sh && ~/get_dog_ip.py`
   - 期望输出：`SN=E03C1CBAAC40A101 ... wlan=<IP>`

2. **验证运动控制并留证**（当前只有口述）
   - 架空或垫起，人在场
   - 用 `example_highlevel.py --read-only` 先确认连接，再按 `take → start walking（零速） → 停止 → release` 顺序
   - **必须保存日志**，避免再次出现"无证据"的情况

3. **补齐基础能力**（写 odom bridge，验证里程计质量）

4. 再决定建图方案（届时需要机身姿态与 odom 质量数据）

---

# 八、已知坑与注意事项（接手必读）

1. **雷达是 MID-360S，不是 MID-360**
   用 `MID360_config.json` 会**静默失败**（无报错、无话题）。必须用 `Mid360s` 块，JSON 块名大小写敏感。

2. **`xfer_format` 的取值**
   `0` = PointCloud2（RViz 可显示）；`1` = CustomMsg（RViz 不能显示）；`2` = PCL PointXYZI，**ROS 2 不支持**。

3. **不要在 `~/uniubi_robot_sdk_py` 目录内 import**
   会加载源码目录下未编译的包，报 `ImportError: cannot import name '_uniubi_robot_motion_py_native'`。请 `cd /tmp` 后运行。

4. **`Error, cannot connect to unix address /tmp/logServer.sock` 是无害警告**，本机没有宇泛日志服务。

5. **MediaBus 视频不可用**，不要在这上面花时间；视频走 RTSP。

6. **机器狗 IP 会漂移**（DHCP）。控制按 SN（`--device-id`），视频用 `~/dog_rtsp.sh` 动态取地址。

7. **SDK `--iface` 必须填 `wlP1p1s0`**，填 `enP8p1s0` 会找不到狗。

8. **RTSP 有会话限制**：连续快速 DESCRIBE 会返回 503，间隔几秒即恢复，不是相机故障。

9. **`colcon version-check` 会联网查 PyPI**，网络差时看起来像卡死。判断 colcon 是否安装请用 `command -v colcon`。

10. **本机 GitHub 访问不稳**（HTTP/2 报错），拉取建议加 `-c http.version=HTTP/1.1`，或改用 codeload tarball。

---

# 九、附：本次采集的关键命令

```bash
# 系统
uname -m; head -1 /etc/nv_tegra_release; lsb_release -ds
cat /proc/device-tree/model; grep TNSPEC /etc/nv_boot_control.conf

# 网络
ip -br addr; ip -4 route; nmcli -f NAME,DEVICE,AUTOCONNECT connection show
ping -c 2 -I 192.168.1.50 192.168.1.194
ip neigh show 192.168.100.114

# SDK
source ~/env_uniubi.sh && cd /tmp
~/venvs/uniubi/bin/pip show uniubi-robot-motion-sdk
~/venvs/uniubi/bin/python -c "import robot_motion_sdk as s; print(s.MEDIA_ENABLED)"
python3 ~/get_dog_ip.py --timeout 25

# 只读查询
python3 ~/uniubi_robot_sdk_py/examples/example_highlevel.py \
  --iface wlP1p1s0 --device-id E03C1CBAAC40A101 --read-only

# 软件包核查
source /opt/ros/humble/setup.bash
for p in slam_toolbox nav2_bringup pointcloud_to_laserscan robot_localization fast_lio; do
  printf "%-28s %s\n" "$p" "$(ros2 pkg list | grep -cx $p)"
done
```

---

**报告结束。**
本报告不含任何推测性的"已完成"。所有【未验证】项在接手后必须重新确认。
