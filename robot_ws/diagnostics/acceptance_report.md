# MID-360 数据验收报告

执行时间：2026-11-12 12:48 ~ 12:52 (Asia/Shanghai)
执行位置：**Jetson 本体**（`nvidia@192.168.100.191`），由 Mac 通过 SSH 远程驱动
启动命令：`ros2 launch livox_ros_driver2 msg_MID360s_launch.py`
配置文件：`/home/nvidia/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json`

## 总结论

> **点云与 IMU 均可稳定读取 —— PASS**

| 验收项 | 结果 | 证据 |
|---|---|---|
| 网络连通（有线口 → 雷达） | **PASS** | ping 4/4，0% 丢包，RTT 0.17–3.3 ms |
| 路由正确性 | **PASS** | `192.168.1.194 dev enP8p1s0 src 192.168.1.50` |
| Wi-Fi SSH 通路未受影响 | **PASS** | 全程 `192.168.100.191` 可连，未重启 NM |
| 设备被发现 | **PASS** | 日志 `dev_type:35, sn:ARMCP9J0039394`，handle 3254888640 |
| 工作模式切换 | **PASS** | `successfully change work mode`、`successfully set lidar attitude` |
| UDP 数据端口绑定 | **PASS** | 56101 / 56201 / **56301** / **56401** 均绑到 `192.168.1.50` |
| 实际 UDP 吞吐 | **PASS** | 网口 23 秒收 **64 MB**（≈2.8 MB/s），此前空跑仅 9 包/3 秒 |
| 点云话题存在 | **PASS** | `/livox/lidar` → `livox_ros_driver2/msg/CustomMsg`，Publisher count 1 |
| IMU 话题存在 | **PASS** | `/livox/imu` → `sensor_msgs/msg/Imu`，Publisher count 1 |
| **点云频率** | **PASS** | `ros2 topic hz` = **9.998 ~ 10.002 Hz**，窗口 65 帧 |
| **IMU 频率** | **PASS** | `ros2 topic hz` = **200.09 ~ 200.19 Hz**，窗口 1609 帧 |
| 点云内容非空 | **PASS** | `point_num = 19968` 点/帧 |
| 点云时间戳推进 | **PASS** | 1794459081.892 → 1794459084.893 |
| IMU 时间戳推进 | **PASS** | 1794459088.791 → 1794459091.364 |
| IMU 数值合理 | **PASS** | 静止时 angular_velocity ≈ (0.025, -0.0005, -0.007) rad/s |
| 重复启动一致性 | **PASS** | 清场后单实例复跑：1 节点 / 2 话题 / 10 Hz / 200 Hz |
| 干净停止 | **PASS** | 停止后 `pgrep` 无残留 |
| RViz2 可视化 | **PASS** | 2026-11-12 13:34 在 Jetson 本机 DP-1（3840×2160）上实测：RViz2 渲染点云正常，31 fps，Global Status: Ok |
| 重启后网络自恢复 | **PASS** | 2026-11-12 13:24 重启，13:26 恢复：`enP8p1s0` 自动持有 `192.168.1.50/24`（由 `mid360-direct` 接管），默认路由仍在 Wi-Fi，ping 雷达 4/4 |
| 点云质量/精度 | **NOT_TESTED** | 未做标定、未建图，未评估点云几何精度 |
| FAST-LIO / SLAM | **NOT_TESTED** | 明确不在本次范围 |
| 宇泛机器狗 SDK | **NOT_TESTED** | 明确不在本次范围 |

## 关键证据（原文摘录）

### 设备发现

```
[livox_ros_driver2_node-1] successfully set lidar attitude, ip: 192.168.1.194
[livox_ros_driver2_node-1] begin to change work mode to 'Normal', handle: 3254888640
[livox_ros_driver2_node-1] successfully change work mode, handle: 3254888640
[livox_ros_driver2_node-1] successfully enable Livox Lidar imu, ip: 192.168.1.194
[livox_ros_driver2_node-1] livox/imu publish use imu format
[livox_ros_driver2_node-1] livox/lidar publish use livox custom format
```

SDK 层（`livox_lidar_quick_start`）交叉验证：

```
Handle detection data, handle:3254888640, dev_type:35, sn:ARMCP9J0039394, cmd_port:56100
Device type:35 point cloud data and IMU data unicast is enabled.
```

### 话题

```
/livox/imu  [sensor_msgs/msg/Imu]
/livox/lidar [livox_ros_driver2/msg/CustomMsg]
```

### 频率

```
[lidar] average rate: 9.998    min: 0.099s max: 0.102s std dev: 0.00067s window: 64
        average rate: 10.002   min: 0.099s max: 0.101s std dev: 0.00054s window: 55
[imu]   average rate: 200.194  min: 0.003s max: 0.007s std dev: 0.00079s window: 1609
```

### 网口吞吐

```
rx_bytes 3123054 -> 68779976   delta = 64118 KB over ~23s
```

### 数据端口

```
UNCONN 192.168.1.50:56101  livox_ros_drive
UNCONN 192.168.1.50:56201  livox_ros_drive
UNCONN 192.168.1.50:56301  livox_ros_drive   <- 点云
UNCONN 192.168.1.50:56401  livox_ros_drive   <- IMU
```

## 第一次失败的原因（保留记录）

首次使用 `msg_MID360_launch.py` + `MID360_config.json` 时：节点正常启动、配置解析成功，
但**没有任何话题**，数据端口未绑定。

- 抓包证明雷达运行正常：每秒回发现包（48B，含 SN）+ 持续广播 push msg（430B）
- 根因：雷达上报 `dev_type = 35 (Mid360s)`，配置声明的却是 `MID360`（类型 9）
- SDK 在 `device_manager.cpp:541` 的类型查表失败处**静默 return**，无任何日志
- 修复：改用官方 `Mid360s` 配置块 → 立即正常

> 教训：`Init lds lidar success!` **不代表**发现雷达；它只说明配置文件解析通过。
> 判断"雷达是否真的连上"，要看 `dev_type`/`sn` 那行，以及 `ros2 topic hz` 的实数。


## 重启持久性专项验证（2026-11-12 13:24 重启）

| 观察项 | 重启后实测 | 结论 |
|---|---|---|
| 开机时间 | `up 2 minutes`，启动于 13:24:10 | 确实重启过 |
| Wi-Fi 地址 | `192.168.100.191/24`（与重启前一致） | PASS |
| 有线地址 | `192.168.1.50/24`，`noprefixroute` | **PASS（自动恢复）** |
| 接管 profile | `mid360-direct`（`nmcli device status` 显示 connected） | PASS |
| 旧的 `Wired connection 1` | 重启后未再出现 | PASS（无 DHCP 干扰） |
| 默认路由 | `default via 192.168.100.1 dev wlP1p1s0` | PASS（未被有线抢占） |
| 到雷达路由 | `192.168.1.194 dev enP8p1s0 src 192.168.1.50` | PASS |
| ping 雷达 | 4/4，0% 丢包，0.96–1.98 ms | PASS |
| 重启后启动驱动 | 设备发现成功，`set lidar attitude` 出现 | PASS |
| 点云 | 10.000 Hz（min 0.097 / max 0.104 s） | PASS |
| IMU | 200.012 Hz（窗口 1603 帧） | PASS |
| 停止后残留 | 无 | PASS |

> 结论：**「192.168.1.50 重启后消失」这个问题已经彻底解决**，无需再手工 `ip addr add`。

备注：重启后 `169.254.0.0/16 dev enP8p1s0` 出现一条链路本地（IPv4LL/avahi）直连路由，与 `192.168.1.0/24` 不冲突，无害。


## RViz2 可视化（2026-11-12 13:34）

环境：Jetson 本机接 DP-1 显示器（3840×2160），Xorg `:0` + GNOME 会话。

**结论：可视化 PASS。**

| 项目 | 结果 |
|---|---|
| RViz2 启动 | 成功，`OpenGl version: 4.6 (GLSL 4.6)` |
| 点云渲染 | 正常，可辨认墙面/天花板/地面/地面物体 |
| 渲染帧率 | **31 fps** |
| Global Status | **Ok** |
| 着色 | 按 `intensity` 通道，`Cost Transformer: Intensity` |

### 关键坑：`xfer_format` 与 ROS 2 的兼容性

| xfer_format | 消息类型 | ROS 2 可用 | RViz2 可渲染 |
|---|---|---|---|
| 0 | `sensor_msgs/msg/PointCloud2`（Livox PointXYZRTL，字段含 x/y/z/intensity） | ✅ | ✅ |
| 1 | `livox_ros_driver2/msg/CustomMsg` | ✅ | ❌（无对应 RViz 插件） |
| 2 | `pcl::PointCloud`（PointXYZI） | ❌ **ROS 2 不支持** | — |

实测 `xfer_format=2` 时驱动打印：

```
error: message type 'pcl::PointCloud' is NOT supported in ROS2,
       please modify the 'xfer_format' field in the launch file
```

且 `/livox/lidar` 话题**完全不出现**（只有 IMU）。

**当前可视化用的是 `xfer_format=0`**，通过 `ros2 run` 直接传参实现，未改动原 launch 文件：

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:=$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json
```

RViz 配置：`
$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/display_point_cloud_ROS2.rviz
`

## 复现步骤

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
# 另开一个终端：
source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash
ros2 topic list -t | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

原始证据文件：`driver_run.log`、`driver_clean.log`、`acceptance_evidence.txt`、`fetch_sources.log`、`installation.log`
