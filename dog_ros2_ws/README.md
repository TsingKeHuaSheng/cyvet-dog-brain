# dog_waypoint —— 让机器狗按既定路径行走

给 `uniubi_motion_bridge` 配的一个最小可用 C++ 业务包。三个节点头：

| 节点 | 作用 |
|---|---|
| `waypoint_recorder` | **教一次**：手动把狗走一遍，按距离采样 `/odom` 存成 CSV |
| `waypoint_follower` | **学一次**：读 CSV，闭环走向每个航点，带安全条件与自动交还控制权 |
| `odom_to_tf` | 把 `/odom` 转成 `odom -> base_link` 的 TF（**上 Nav2 / RViz 必需**） |

---

## 0. 一条命令跑通（保底方案）

三个脚本把"环境变量 + bridge + 业务节点"全包住了，**不用再手动 export 任何东西**。

```bash
chmod +x ~/dog_ros2_ws/scripts/*.sh
```

### 0.1 编译（只做一次）

```bash
cd ~/dog_ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select dog_waypoint
source install/setup.bash
```

### 0.2 回放：**一条命令**

```bash
~/dog_ros2_ws/scripts/dog_replay.sh
```

默认走 `paths/first_test.csv`（只往前 0.5 m），速度 `0.12 m/s`。它会：

```text
启动 bridge → 等 /odom → /motion/start_action 取权 → 闭环走到 0.5 m
→ 停车 → stop_action → release_control → 自己退出
```

看到 `全部航点完成` + `已释放控制权，退出` 就算通。**这一条命令就验证了整条链路**：
取权、里程计、闭环、到点、停车、交权。

走自己的路径：

```bash
~/dog_ros2_ws/scripts/dog_replay.sh ~/dog_paths/my.csv
V_MAX=0.10 W_MAX=0.3 ~/dog_ros2_ws/scripts/dog_replay.sh ~/dog_paths/my.csv
```

### 0.3 录点：一条命令 + 一条驱动命令

终端 1（起 bridge + 记录器）：

```bash
~/dog_ros2_ws/scripts/dog_record.sh ~/dog_paths/my.csv
```

终端 2（唯一的"第二条命令"，负责驱动）：

```bash
# 先取权（录制期间必须有 Walk 动作才有 /odom）
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"

# 前进；Ctrl+C 停这条 pub，bridge 会在 500ms 后自动清零速度
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}'

# 转向
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.25}}'

# 停
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}'
```

走完后：终端 2 `Ctrl+C` → 终端 1 `Ctrl+C`（CSV 已落盘，每点都写）→ 用 0.3 的命令回放。

> 终端 2 里的命令不需要 `source` 环境：它们只和**正在运行的 bridge** 通信，
> 而 bridge 已经在终端 1 的 launch 里了。如果提示找不到话题，先在那条命令前
> 执行 `source ~/dog_ros2_ws/scripts/dog_env.sh`。

### 0.4 出问题先跑体检

```bash
~/dog_ros2_ws/scripts/dog_check.sh
```

它会告诉你：包找到了没、`/cmd_vel` `/odom` `/motion/status` `/battery_state` 在不在、
`/odom` 有没有真数据、`/motion/*` 服务列出来了没。

---

## 1. 先理解编程模型：你在第几层

```text
第 3 层  任务/路线        Nav2 全局规划，或本包的航点列表
          ↑ 输出目标点 / 速度
第 2 层  行为（C++ 节点）  ← 你真正要写的东西：状态机 + 安全条件
          ↑ 输出 /cmd_vel
第 1 层  底盘接口          uniubi_motion_bridge: /cmd_vel + /motion/*
          ↑
第 0 层  机器狗固件        DDS / RPC（robotServer, domain 42）
```

**关键点：`/cmd_vel` 只是"速度参数入口"，它不闭环、不取权、不启动动作。**
所以业务代码的骨架永远是：

```text
等依赖就绪 → /motion/start_action（自动取权）→ 循环{读 /odom → 判断 → 发 /cmd_vel}
          → /motion/stop_action → /motion/release_control
```

`waypoint_follower.cpp` 就是这个骨架的完整实现，可以直接改成你的业务逻辑。

---

## 2. 编译

```bash
cd ~/dog_ros2_ws          # 本目录
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash    # 提供 uniubi_motion_bridge 的 msg/srv
colcon build --packages-select dog_waypoint
source install/setup.bash
```

前置：`uniubi_motion_bridge` 的 workspace 已经编译好且被 source 过（否则找不到
`uniubi_motion_bridge/srv/StartMotionAction` 这个类型）。

---

## 3. 运行整套

三个终端（环境变量三个终端都要设，同一套）：

```bash
# 公共环境（每个终端都要执行）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/dog_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/nvidia/dds-direct.xml   # 或 dds-public.xml
```

**终端 A：bridge**

```bash
ros2 launch uniubi_motion_bridge motion_bridge.launch.py \
  device_id:=E03C1CBAAC40A101
```

**终端 B：让狗走路径**

```bash
mkdir -p ~/dog_paths

ros2 run dog_waypoint waypoint_follower --ros-args \
  -p path_file:=$HOME/dog_paths/path.csv \
  -p v_max:=0.20 -p w_max:=0.5 \
  -p pos_tolerance:=0.15 \
  -p max_duration_s:=90.0 \
  -p battery_min:=0.20
```

**终端 C：可选，看 TF 和状态**

```bash
ros2 run dog_waypoint odom_to_tf
ros2 topic echo /motion/status
```

> 取权前**遥控器必须断开**（长按 `M` 键，听到"遥控器连接已断开"），否则 `start_action` 会失败。

---

## 4. 录点回放（最快让狗"学会"一条路径）

### 4.1 录

```bash
ros2 run dog_waypoint waypoint_recorder --ros-args \
  -p output_file:=$HOME/dog_paths/my_path.csv \
  -p min_step_m:=0.20
```

另一个终端让它慢慢走（先取权、再手动发速度）：

```bash
ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
  "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"

# 前进 0.15 m/s（Ctrl+C 停这条 pub 后，bridge 会在 cmd_vel_timeout_ms=500ms 后自动清零速度）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}'

# 转弯
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'
```

走完 Ctrl+C 结束记录器，CSV 已落盘（每采一点就写一次）。

### 4.2 回放

```bash
ros2 run dog_waypoint waypoint_follower --ros-args \
  -p path_file:=$HOME/dog_paths/my_path.csv
```

### 4.3 为什么不直接用时间回放

时间回放对速度误差没有纠正能力，5 米之后就会偏出半米。按 `/odom` 位置闭环比按时间回放稳得多，
这也是本包的核心。

---

## 5. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `path_file` | 必填 | 航点 CSV |
| `v_max` | 0.25 | 前进速度上限 m/s |
| `w_max` | 0.6 | 转向角速度上限 rad/s |
| `k_rho` | 0.8 | 距离比例增益 |
| `k_alpha` | 1.5 | 角度比例增益 |
| `pos_tolerance` | 0.15 | 到点判定半径 m |
| `yaw_tolerance` | 0.15 | 末点朝向容差 rad |
| `max_duration_s` | 120 | 总时长上限，超时自动停车 |
| `battery_min` | 0.15 | 电量下限（0~1），低于则中止 |
| `odom_timeout_s` | 1.0 | 里程计超时判定 |
| `control_rate_hz` | 10 | 控制循环频率（≥ 5，bridge 的 cmd_vel 超时是 500ms） |
| `loop` | false | 走完是否循环 |
| `start_action` | walking | 启动的动作名 |

录点节点参数：`output_file`、`min_step_m`（默认 0.2）、`min_yaw_step_rad`（默认 0.35）。

---

## 6. 必读的接口语义（踩过的坑）

| 事实 | 后果 |
|---|---|
| `/cmd_vel` **不取权、不启动动作** | 必须先 `/motion/start_action`，否则发了也没用 |
| `/cmd_vel` 超时 500ms 后 bridge 自动清零速度 | 控制循环必须 ≥ 5 Hz，不能"发一次就不管" |
| `stop_action` ≠ 释放控制权 | 结束业务要显式 `/motion/release_control` |
| `/odom` 是设备端已累计的 position/yaw | **上层不要再积分**，直接用 |
| `/odom` 没有 TF | 需要本包的 `odom_to_tf`，否则 Nav2/RViz 无法建树 |
| 里程计只在 Walk 动作期间有效 | 别在 `standing` 时指望有 `/odom` |
| `start_action` 返回成功 ≠ 动作已完成 | 以 `/motion/status.current_action` 为准 |
| 遥控器连着就无法取权 | 先断开再跑业务 |
| 首次取权会先把 master 切回小脑再申请会话 | 第一次 `start_action` 会慢一点，属正常 |

---

## 7. 真正成熟的做法：Nav2 + 地图 + 定位

本包的航点回放适合"固定路线、短距离、无动态障碍"。要做得更正式，用 **Nav2**：

```text
你已有的资产（ws_slam）：
  Livox MID360 → FAST-LIO2 里程计 → PGO 建图 → localizer 重定位
  已存地图：~/robot_ws/maps/{run1,run2,run3,office1}/map.pcd

Nav2 需要补齐的：
  map_server（把 pcd 转成 2D 占据栅格或直接用 3D 定位）
  + planner（全局路径）
  + controller（DWB/MPPI，输出 /cmd_vel）
  + TF 树：map -> odom -> base_link   ← 用本包 odom_to_tf 补最后一环
```

关键落地点：

1. **Nav2 的 `/cmd_vel` 直接喂给 bridge** —— 把 uniubi bridge 当成一个普通底盘驱动，
   Nav2 不需要知道你底下是 DDS 还是串口。
2. **TF 必须自洽**：`map->odom` 由定位模块给，`odom->base_link` 用 `odom_to_tf`，
   `base_link->...` 由 URDF/`robot_state_publisher` 给。
3. **速度限幅放在 Nav2 侧**（`max_vel_x`、`max_vel_theta`），不要指望 bridge 限幅——
   文档明确它不做动作范围限幅。
4. **代价地图**：狗是四足，`footprint` 要按实际体宽留足余量。

升级顺序建议：本包航点闭环先跑通（验证控制权、里程计、停车、回收链路）→ 再接 Nav2。

---

## 8. 扩展方向

- **条件触发**：把 `driving()` 里的安全判断改成你的业务规则（到某区域减速、看到人停、
  电量分段降速）——这就是"在具体条件下执行相应指令"。
- **多段路径**：CSV 里加一列速度上限，或做多文件串联。
- **与视觉联动**：订阅 `uniubi_media_driver` 的 JPEG topic，命中目标后在当前航点暂停。
- **状态机 DSL**：路径点 + 动作（停 3 秒、播报、拍照）做成 YAML，节点解释执行。
