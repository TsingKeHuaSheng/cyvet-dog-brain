# Cyvet 机器狗大脑 · 团队开发仓库

UniUbi 灵猫 Cyvet 四足机器人（型号 CYVET-CREATOR）二次开发代码。
算力板换成 **Jetson Orin Nano 8GB**，雷达 **Livox MID-360S**，系统 Ubuntu 22.04 + ROS 2 Humble。

> 目标：建图 → 定位 → 画线/规划 → 循线行走（A 点到 B 点）。
> 本仓库只放**我们自己写的代码、必要配置和文档**，不放第三方源码、虚拟环境、编译产物和模型权重。
> 仓库根目录就是 Jetson 上的 `/home/nvidia`，所以下面所有路径都能和机器上的实际路径一一对应。

---

## 1. 环境

| 项目 | 值 |
|---|---|
| 机器 | UniUbi Cyvet 四足（小脑板跑 `robotServer`） |
| 大脑 | NVIDIA Jetson Orin Nano 8GB，`nvidia@nvidia-desktop`，aarch64 |
| 系统 | Ubuntu 22.04 + ROS 2 Humble |
| 雷达 | Livox MID-360S（**不是** MID-360），以太网直连 Jetson |
| 通信 | Jetson 用 Wi-Fi 连狗：网卡 `wlP1p1s0` = `192.168.100.191/24`；雷达专线 `enP8p1s0` = `192.168.1.50/24` |
| 狗 IP | `192.168.100.114`（**会漂**，用 `./get_dog_ip.py` 现查） |
| SDK | UniUbi HighLevel SDK，C++ + Python 两套，`aarch64_host` 平台 |

环境一把梭：

```bash
source ~/env_uniubi.sh      # 设 UNIUBI_SDK_ROOT / LD_LIBRARY_PATH / venv / IFACE / SN
```

---

## 2. 代码结构（仓库里有什么）

```text
~/                                  ← 仓库根 = Jetson 家目录
├── README.md
├── .gitignore                      白名单式：默认全忽略，只放行下面这些
├── .gitattributes                  统一 LF 换行，点云按二进制处理
│
├── docs/                           文档（架构、排错、交接、操作手册）
│
├── env_uniubi.sh                   ★ 环境变量（V1.00.000）
├── env_uniubi_v100001.sh             备用版本 V1.00.001
├── dog_console.sh                  ★ HighLevel 交互控制台（封装 venv + iface + SN）
├── dog_rtsp.sh                       自动查 IP 拼 RTSP 地址
├── get_dog_ip.py                   ★ 查狗当前 IP
│
├── ros2_ws/src/                    工作区入口：3 个**相对**软链，指向三方 ~/uniubi_ros2
│   ├── uniubi_motion_bridge  → ../../uniubi_ros2/src/uniubi_motion_bridge
│   │                                 /motion/* 服务 ↔ SDK RPC 翻译层（20 个文件）
│   ├── uniubi_motion_client  → ../../uniubi_ros2/src/uniubi_motion_client
│   └── uniubi_robot_msgs     → ../../uniubi_ros2/.deps/uniubi_robot_msgs（68 个 msg/srv）
│
├── robot_tools/                    ★ 自研：office4 业务层（1 MB，全部手写）
│   ├── office4_drive/                drive.launch.py、bridge.bash、dog_env.bash、
│   │                                 command_link.bash、base_odom.py、controller.yaml
│   ├── office4_route/                取权/释放/启动脚本
│   ├── office4_live_obstacles/       高度滤波避障
│   ├── navigation_preview/           栅格预览、planner.yaml
│   ├── chair_preview/                椅子场景预览
│   ├── dogroute/                     dogroute 入口
│   ├── docs/                         建图/定位/循线操作笔记
│   └── misc/                         TF 图、网络排查记录
│
├── ws_route/src/office4_route/      ★ 自研：画线循线主包（本项目的核心创新点）
│   ├── src/route_controller.cpp       RPP 循线控制器（Nav2 插件）
│   ├── src/route_tool.cpp             路线工具
│   ├── scripts/editor.py              画线编辑器（RViz 插件）
│   ├── scripts/walk.py                读路线 → FollowPath → /cmd_vel
│   ├── scripts/route_map.py           路径加密（≤5 cm 间距）
│   ├── scripts/obstacles.py           障碍点高度滤波
│   ├── scripts/*.bash                 start / walk / stand / session_guard
│   ├── config/                        controller.yaml、office4.launch.py、RViz
│   └── test/                          单元测试
│
├── dog_ros2_ws/                    ★ 自研：航点"教一次/学一次"
│   ├── src/dog_waypoint/              waypoint_recorder / waypoint_follower / odom_to_tf
│   ├── scripts/                       dog_replay.sh、dog_record.sh、dog_check.sh、dog_env.sh
│   └── src/dog_waypoint/paths/        示例航点 CSV
│
├── nav2_nav/                       ★ 自研：Nav2 参数与调试工具
├── cyvet_demo/                     ★ 自研：C++ SDK 最小示例（build.sh / run.sh）
├── ws_livox/src/livox_pc2_bridge/  ★ 自研：Livox CustomMsg → PointCloud2（给 RViz 看）
│
├── ws_slam/src/FASTLIO2_ROS2/        三方：FAST-LIO2 + 回环 + 重定位 + 地图优化
│                                     （体积小、必须同版本才能编译，故随仓库提供）
│
├── robot_ws/maps/                    地图：office4（当前场地）+ run1~3 + office1~3
├── robot_ws/diagnostics/             诊断报告、交接文档、验收证据
├── dog_paths/                        采集到的航点 CSV
└── third_party/ws_pct_cyvet/         ws_pct（gitee 三方）的本地定制补丁与配置
```

---

## 3. 仓库里**没有**的东西（以及怎么补上）

这些东西合计约 3.5 GB，全部靠安装脚本 / 克隆 / apt 恢复，不进 Git：

| 缺失项 | 大小 | 怎么补 |
|---|---|---|
| `uniubi_robot_sdk/`（C++ SDK） | 71 MB | `git clone https://github.com/uniubi-ai/uniubi_robot_sdk.git ~/uniubi_robot_sdk` |
| `uniubi_robot_sdk_py/`（Python SDK） | 8 MB | `git clone https://github.com/uniubi-ai/uniubi_robot_sdk_py.git ~/uniubi_robot_sdk_py` |
| `uniubi_ros2/` | 3 MB | **必需**（`ros2_ws/src` 的三个软链指向它）：`git clone https://github.com/uniubi-ai/uniubi_ros2.git` |
| `ws_livox/src/livox_ros_driver2/` | 23 MB | `git clone https://github.com/Livox-SDK/livox_ros_driver2.git`（MID-360S 需较新分支） |
| `robot_ws/Livox-SDK2/` | 29 MB | `git clone https://github.com/Livox-SDK/Livox-SDK2.git`，按 README 编译安装 |
| `ws_pct/`（3D 全局规划，gitee 三方） | 845 MB | `git clone https://gitee.com/fdsf3e2342/3d_navi_ros2.git ws_pct`，再把 `third_party/ws_pct_cyvet/` 里的补丁与 `cyvet*.yaml/launch.py` 贴回去 |
| `venvs/`（Python 3.10 虚拟环境） | 42 MB | `python3 -m venv ~/venvs/uniubi` + `pip install -r` 对应 SDK 的 requirements |
| `JEP/`、`jetson-inference/`、`ultralytics/`、`Viewer2_2.6.2_Linux/` | 2.1 GB | 视觉/工具链，非本仓库职责；需要时按各自官方 README 装 |

> `ws_pct` 是**实验性**的 3D 全局规划器，主线循线走的是 `ws_route` + Nav2，所以不整包入库。

---

## 4. 从零到能跑（新同事 / 新机器）

```bash
# 1) 克隆到「家目录」——注意是 ~/，不是 ~/xxx，路径必须和文档一致
cd ~
git clone git@github.com:<你>/cyvet-dog-brain.git .    # 或 https 形式
#   若 ~ 下已有同名文件会冲突，先备份再克隆

# 2) 补齐三方依赖（见 §3 表格）——至少要有 uniubi_ros2，否则 ros2_ws/src 下是断链
git clone https://github.com/uniubi-ai/uniubi_ros2.git ~/uniubi_ros2
ls -l ~/ros2_ws/src            # 三个软链不应是红色断链

# 3) 编译
source /opt/ros/humble/setup.bash
cd ~/ros2_ws   && colcon build --symlink-install && cd -
cd ~/ws_route  && colcon build --symlink-install && cd -
cd ~/ws_slam   && colcon build --symlink-install && cd -
cd ~/dog_ros2_ws && colcon build --packages-select dog_waypoint && cd -

# 4) 起环境
source ~/env_uniubi.sh
python3 ~/get_dog_ip.py            # 看狗现在 IP 是多少

# 5) 跑一遍最小闭环（取权 → 走 0.5 m → 停车 → 交权）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
~/dog_ros2_ws/scripts/dog_replay.sh
```

---

## 5. 这份 `.gitignore` 怎么用

它是**白名单**式：`/*` 先把家目录全忽略，再 `!/目录/` 逐项放行。

- 想加一个要上传的目录 → 在 §2 加一行 `!/你的目录/`，然后 `git status` 复核。
- 想确认某文件为什么没被跟踪 → `git check-ignore -v 路径`。
- **绝对不要**在 `~` 下执行 `git clean -fdx`（会删掉整个家目录）。
- 提交前自检：`git status --short` 里不该出现 `build/ install/ log/ __pycache__/`。

---

## 6. 协作约定

| 事项 | 约定 |
|---|---|
| 分支 | `main` 只放能跑的版本；开发走 `feat/xxx`、`fix/xxx` |
| 提交信息 | 中文可，前缀 `feat:` / `fix:` / `docs:` / `chore:` |
| 二进制 | 地图 `.pcd` 可以进（单个 < 5 MB）；录包 `.bag/.mcap`、权重 `.pt/.onnx` 不进 |
| 现场参数 | 狗的 IP / SN 写在 `robot_tools/office4_drive/dog_env.bash` 与 `motion_bridge.yaml`，换场地只改这两处 |
| 机密 | WiFi 密码、SSH 私钥、GitHub Token 一律不入库（已在 `.gitignore` §4 兜住） |
| 换行 | 一律 LF（`.gitattributes` 已强制），Windows 上编辑 `.sh` 记得别存成 CRLF |

---

## 7. 已知的坑（踩过，写下来）

1. **别在 `~/uniubi_robot_sdk_py` 目录里 import** 该 SDK，会加载源码目录下未编译的包，报 `cannot import name '_uniubi_robot_motion_py_native'`。`dog_console.sh` 里先 `cd /tmp` 就是干这个的。
2. 网卡必须是 **`wlP1p1s0`**，`enP8p1s0` 是雷达专线，选错永远发现不到狗。
3. 狗 IP 会漂，脚本里别写死；用 `get_dog_ip.py`。
4. 两个 ROS 域互相不可见：域 0（导航，FastDDS）与域 42（SDK，CycloneDDS）。`echo $ROS_DOMAIN_ID` 先确认自己站在哪个域。
5. 摄像头 RTSP 并发会 503，一次只开一路。
6. 平台常数永远是 `aarch64_host`（外部 ARM64 主机），不是 `aarch64`。
7. 仓库里有 11 个**软链**（`ros2_ws/src/*` 指向三方 `~/uniubi_ros2`，`robot_tools/*` 指向 `ws_route` 的脚本）。已全部改成相对路径，clone 到任何 `~` 下都能解析；看到红色断链就是对应依赖没装。
