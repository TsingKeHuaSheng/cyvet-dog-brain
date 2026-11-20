# MID-360 配置与启停速查

## 1. 地址与标识

| 项目 | 值 |
|---|---|
| 雷达序列号 | `ARMCP9J0039394` |
| 雷达 IP | `192.168.1.194/24` |
| 雷达 MAC | `34:91:f0:e4:0d:c8` |
| 雷达网关（内部配置） | `192.168.1.1`（雷达自带，我们不用它） |
| **SDK 上报设备类型** | **35 = `kLivoxLidarTypeMid360s`**（不是 9） |
| Jetson 有线口 | `enP8p1s0`，MAC `3c:6d:66:c3:ad:c8`，100Mb/s Full |
| Jetson 有线 IP | `192.168.1.50/24` |
| Jetson Wi-Fi | `wlP1p1s0`，`192.168.100.191/24`，网关 `192.168.100.1` |

> ⚠️ **最重要的一条**：这台机器上报的是 **Mid360s（35）**，不是普通 MID360（9）。
> 用 `MID360_config.json` 会**静默失败**（无任何报错，只是没有数据）。**必须用 `Mid360s` 那份配置。**

## 2. 网络配置（持久化）

NetworkManager 连接名：**`mid360-direct`**

- 配置文件：`/etc/NetworkManager/system-connections/mid360-direct.nmconnection`
- 绑定接口：`enP8p1s0`
- 地址：`192.168.1.50/24`
- 网关：**无**；DNS：**无**；`ipv4.never-default = yes`；`route-metric = 700`
- IPv6：`disabled`
- `autoconnect = yes`，优先级 10，重试次数 0（不限）

Wi-Fi 的 `清科华胜` profile **未做任何修改**。

查看/恢复命令：

```bash
nmcli connection show mid360-direct          # 看配置
ip -br addr show enP8p1s0                    # 看地址
ip -4 route get 192.168.1.194                # 必须 dev enP8p1s0 src 192.168.1.50
```

手工恢复（万一 profile 丢失）：

```bash
sudo nmcli connection add type ethernet con-name mid360-direct ifname enP8p1s0 \
  ipv4.method manual ipv4.addresses 192.168.1.50/24 \
  ipv4.never-default yes ipv4.route-metric 700 ipv6.method disabled \
  connection.autoconnect yes
sudo nmcli connection up mid360-direct
```

## 3. 雷达配置文件

**实际生效的文件（安装副本）：**

```
/home/nvidia/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json
```

**源文件（改这里，然后重建）：**

```
/home/nvidia/ws_livox/src/livox_ros_driver2/config/MID360s_config.json
```

关键内容：

```json
{
  "lidar_summary_info": { "lidar_type": 8 },
  "Mid360s": {
    "lidar_net_info": {
      "cmd_data_port": 56100, "push_msg_port": 56200,
      "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500
    },
    "host_net_info": [{
      "host_ip": "192.168.1.50",
      "cmd_data_port": 56101, "push_msg_port": 56201,
      "point_data_port": 56301, "imu_data_port": 56401, "log_data_port": 56501
    }]
  },
  "lidar_configs": [ { "ip": "192.168.1.194", "pcl_data_type": 1, "pattern_mode": 0, ... } ]
}
```

### 端口方向

| 用途 | 雷达侧 | Jetson 侧 |
|---|---|---|
| 控制指令 | 56100 | 56101 |
| 状态推送 | 56200 | 56201 |
| **点云** | 56300 | **56301** |
| **IMU** | 56400 | **56401** |
| 日志 | 56500 | 56501 |
| 发现（广播） | 56000 | 56000 |

### schema 差异（易踩坑）

| | `MID360s_config.json` / SDK sample | `MID360_config.json` |
|---|---|---|
| JSON 块名 | `"Mid360s"` → 类型 **35** | `"MID360"` → 类型 **9** |
| `host_net_info` | **数组**，元素含 `host_ip` | **对象**，含 `cmd_data_ip`/`point_data_ip`… |
| 多播 | 可选 `multicast_ip`，不填=单播 | 无此字段 |

**不要混用**：块名、`host_net_info` 结构、类型码三者必须自洽。

## 4. 启动

先 source（每个新终端都要）：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
```

**两个 launch 文件，按用途选一个：**

| 用途 | 命令 | xfer_format | /livox/lidar 类型 |
|---|---|---|---|
| **RViz 看点云** | `ros2 launch livox_ros_driver2 msg_MID360s_pc2_launch.py` | 0 | `sensor_msgs/msg/PointCloud2` |
| **给 FAST-LIO** | `ros2 launch livox_ros_driver2 msg_MID360s_launch.py` | 1 | `livox_ros_driver2/msg/CustomMsg` |

- `msg_MID360s_launch.py` 是**官方原文件，未改动**。
- `msg_MID360s_pc2_launch.py` 是本次**新增**，与原文件**只差 `xfer_format=0` 一行**，
  目的是让 RViz2 能直接显示（RViz2 没有解析 CustomMsg 的插件）。
- 两个文件都已放在源目录 `launch_ROS2/` 和安装目录 `install/.../share/livox_ros_driver2/launch_ROS2/`，
  重新 `./build.sh humble` 后新文件会被重新安装。

> ⚠️ 注意文件名是 **`MID360s`**（带 s）、**`Mid360s`**（配置块），不是 `MID360`。
> 用 `MID360` 那份会静默失败（设备类型 9 vs 实际 35）。

### RViz 启动

```bash
rviz2 -d ~/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/display_point_cloud_ROS2.rviz
```

该配置已预设：`Fixed Frame = livox_frame`、`PointCloud2` 订阅 `/livox/lidar`、按 `intensity` 着色。
若手动开空白 rviz2，必须自己把 `Fixed Frame` 改成 `livox_frame`，否则状态栏报 `Fixed Frame [map] does not exist`。

后台运行（SSH 断开不丢）：

```bash
setsid nohup bash -c 'source /opt/ros/humble/setup.bash; source ~/ws_livox/install/setup.bash; \
  exec ros2 launch livox_ros_driver2 msg_MID360s_launch.py' \
  > ~/robot_ws/diagnostics/driver_run.log 2>&1 < /dev/null &
```

## 5. 话题

| 话题 | 类型 | 频率 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | ~10 Hz |
| `/livox/imu` | `sensor_msgs/msg/Imu` | ~200 Hz |

`frame_id` = `livox_frame`，`publish_freq` = 10.0，`multi_topic` = 0，`xfer_format` = 1（自定义 CustomMsg）。

想让 `/livox/lidar` 变成标准 `sensor_msgs/msg/PointCloud2`：把 launch 文件里 `xfer_format` 改成 `0`
（PointXYZRTL）或 `2`（PCL PointXYZI），改完需重新 `source` 并重启 launch。

## 6. 停止

```bash
pkill -INT -f livox_ros_driver2_node      # 先温和中断
pkill -TERM -f livox_ros_driver2_node     # 仍在则 TERM
pkill -TERM -f 'ros2 launch livox_ros_driver2'
```

> 实测经验：`SIGINT` 有时不足以让节点退出，需补一次 `TERM`。
> 残留进程会导致同名节点重复（`ros2 node list` 出现两条 `/livox_lidar_publisher`），启动前先确认干净：
> `pgrep -af livox_ros_driver2`

## 7. 快速自检

```bash
ip -4 route get 192.168.1.194          # 应为 dev enP8p1s0 src 192.168.1.50
ping -I 192.168.1.50 -c 4 192.168.1.194
ros2 topic list -t | grep livox        # 应有 /livox/lidar 与 /livox/imu
ros2 topic hz /livox/lidar             # 应约 10 Hz
ros2 topic hz /livox/imu               # 应约 200 Hz
```

## 8. 开机自启（2026-11-16 改为 systemd 服务）

### 8.1 服务说明

| 项目 | 内容 |
|---|---|
| 服务名 | `livox-lidar.service` |
| 文件 | `/etc/systemd/system/livox-lidar.service` |
| 状态 | `enabled` + `active` |
| 模式 | **CustomMsg**（`msg_MID360s_launch.py`，供 FAST-LIO 使用） |
| 窗口 | **无**，后台运行 |
| 日志 | `journalctl -u livox-lidar` |

ExecStart 里做的事（systemd 不读 `.bashrc`，必须自己 source）：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

启动前置：最多 60 秒等 `enP8p1s0` 拿到 `192.168.1.50`。

失败重启：`Restart=on-failure`，间隔 10 秒，不限次数。

### 8.2 常用命令

```bash
systemctl status livox-lidar          # 看状态
journalctl -u livox-lidar -f          # 实时日志
sudo systemctl stop livox-lidar       # 停止
sudo systemctl start livox-lidar      # 启动
sudo systemctl restart livox-lidar    # 重启
sudo systemctl disable livox-lidar    # 取消开机自启
```

### 8.3 实测记录

**2026-11-16 10:40 重启测试**

| 项目 | 结果 |
|---|---|
| 开机时间 | 10:40:58 |
| 服务自启 | 10:41:16（**开机 18 秒后**）✅ |
| 终端窗口 | 无 ✅ |
| `enP8p1s0` | 自动获得 `192.168.1.50` ✅ |
| `/livox/lidar` | CustomMsg，9.99 Hz ✅ |
| `/livox/imu` | 199.98 Hz ✅ |

### 8.4 ⚠️ 雷达未连接时的表现

**情况 A：服务没起来**

```bash
systemctl is-active livox-lidar   # 返回 inactive
```

**情况 B：服务在跑，但雷达没接或没识别（更危险）**

服务状态仍是 `active`，但**一个话题都不发布**。

原因：驱动找不到设备时不退出，进程常驻。

判据：**只有 `ros2 topic hz /livox/lidar` 稳定输出 ~10 Hz 才算真的正常。**

### 8.5 与手动 launch 的冲突

服务固定使用 CustomMsg 模式。若想用 pc2 模式（RViz 直接看原始点云），
**必须先停服务**，否则端口 56301 冲突：

```bash
sudo systemctl stop livox-lidar
ros2 launch livox_ros_driver2 msg_MID360s_pc2_launch.py
```

用完恢复：

```bash
sudo systemctl start livox-lidar
```

### 8.6 已删除的内容

| 删除项 | 原因 |
|---|---|
| `~/.config/autostart/start-lidar.desktop` | 改用 systemd，不再弹终端 |
| `~/robot_ws/scripts/start_lidar.sh` | 同上 |
| `/etc/systemd/system/livox-mid360.service` | 早期版本，已废弃 |
