# FAST-LIO 建图环境（2026-11-13 安装并实测）

## 已安装

| 组件 | 位置 | 版本/来源 |
|---|---|---|
| FASTLIO2_ROS2 | `~/ws_slam/src/FASTLIO2_ROS2` | liangheming/FASTLIO2_ROS2 main |
| 编译产物 | `~/ws_slam/install/` | fastlio2 / pgo / localizer / hba / interface |
| Sophus | `/usr/local/include/sophus` | 1.22.10（纯头文件，未编译测试） |
| GTSAM | apt `ros-humble-gtsam` | 4.2.0 |
| 雷达驱动 | `~/ws_livox` | 已有（CustomMsg / pc2 两种 launch） |

## 实测结果（雷达静止在桌面）

| 项目 | 结果 |
|---|---|
| `/fastlio2/lio_odom` | **10.0 Hz** |
| `/fastlio2/world_cloud` | **10.0 Hz** |
| 静态位姿漂移 | 位置 ≈ (-0.017, 0.002, -0.001) m，姿态 ≈ 单位四元数 |
| lio_node CPU | **30% 单核** |
| lio_node 内存 | **116 MB** |
| 系统可用内存 | 4.9 GB |

→ Orin Nano 8GB 跑 FAST-LIO 很轻松。

## 运行方式

### 建图（一键）

```bash
~/robot_ws/scripts/start_mapping.sh
```

脚本会：停掉已在运行的驱动 → 起雷达(CustomMsg) → 起 FAST-LIO + 回环 → 打开 RViz。
**走完一圈后按 Ctrl+C 结束。**

### 保存地图（建图仍运行时，另开终端）

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_slam/install/setup.bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/nvidia/robot_ws/maps/run1', save_patches: true}"
```

产出：`~/robot_ws/maps/run1/` 下的 pcd 与位姿文件。

### 只看点云（不建图）

```bash
~/robot_ws/scripts/start_lidar.sh   # pc2 模式，RViz 里 Fixed Frame 选 livox_frame
```

## 关键话题

| 话题 | 内容 |
|---|---|
| `/fastlio2/lio_odom` | 里程计（frame: lidar → body） |
| `/fastlio2/world_cloud` | 世界系点云（建图结果） |
| `/fastlio2/body_cloud` | 机体系点云 |
| `/fastlio2/lio_path` | 轨迹 |
| `/pgo/loop_markers` | 回环标记 |

## 配置文件

| 文件 | 关键项 |
|---|---|
| `fastlio2/config/lio.yaml` | `imu_topic: /livox/imu`、`lidar_topic: /livox/lidar`、`gravity_align: true`、`det_range: 60` |
| `pgo/config/pgo.yaml` | 输入 `/fastlio2/body_cloud` + `/fastlio2/lio_odom`，回环半径 1.0 m |

## 注意事项

1. **两种驱动模式不能同时跑**（端口 56301 冲突）
   - `msg_MID360s_launch.py` → CustomMsg → **FAST-LIO 用这个**
   - `msg_MID360s_pc2_launch.py` → PointCloud2 → **RViz 看云用这个**
2. `start_mapping.sh` 已自动处理冲突，但开机自启的终端会停在那儿，属正常
3. 雷达静止时建图会把同一片点云反复叠加，**必须在走动过程中建图**
4. 回环检测需要**回到起点附近**才触发（`loop_search_radius: 1.0`）
