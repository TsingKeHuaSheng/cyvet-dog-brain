# 后续工作与对接条件

## 当前状态一句话

雷达 → Jetson 的**点云与 IMU 已打通并实测稳定**（10 Hz / 200 Hz）。
尚未做：重启验证、可视化、SLAM、机器狗对接。

## 一、建议先补的验证（都很轻）

| # | 事项 | 为什么 | 怎么做 |
|---|---|---|---|
| 1 | ~~重启一次，确认 `192.168.1.50` 自动回来~~ | **已完成 2026-11-12 13:24**，自动恢复 PASS | 见 `acceptance_report.md` 重启专项 |
| 2 | ~~确认重启后驱动能直接起~~ | **已完成**，点云 10.000 Hz / IMU 200.012 Hz | 见 `acceptance_report.md` |
| 3 | RViz2 可视化（可选） | 肉眼确认点云形状正常 | 需要 Jetson 接显示器，或 Mac 侧配 X 转发 / 用 Foxglove 通过 ROS 2 远程看 |
| 4 | 让 `/livox/lidar` 输出 `sensor_msgs/PointCloud2` | 部分下游节点只吃标准点云 | launch 里 `xfer_format` 改 0 或 2，重建/重启 |

## 二、接 FAST-LIO 的条件

前置条件（当前都已满足）：

- [x] `/livox/lidar` 有数据，10 Hz
- [x] `/livox/imu` 有数据，200 Hz
- [x] 两者时间戳同源（都来自雷达内部时钟）
- [x] 磁盘余量 83 GB（录 rosbag 够用）
- [x] 内存可用 ~5.1 GB

需要注意的点：

1. **FAST-LIO 的原生输入是 `livox_ros_driver2/msg/CustomMsg`** —— 现在正是这个格式，可以直接用（`xfer_format=1` 保持不变最省事）。
2. FAST-LIO 需要 `livox_ros_driver2` 的 msg 定义，即编译时必须先 `source ~/ws_livox/install/setup.bash`。
3. 首次跑 FAST-LIO 前建议先录一段 rosbag 存档，便于反复调参。
4. 雷达外参先按零处理，装到狗身上后**必须重新标定**（雷达↔IMU↔机体）。

~~等确认"重启后能自动恢复"~~ → **已确认通过**。现在可以开始考虑 FAST-LIO 了。

## 三、接宇泛机器狗 SDK 的条件

目前**完全没碰**，也**没有任何该机器狗的型号/SDK/接口资料**。要推进需要先拿到：

1. **机器狗确切型号**（宇泛四足，具体是哪个型号）
2. **SDK 名称与版本**、是否提供 Linux/aarch64 的库
3. **通信方式**：Wi-Fi（IP/端口/协议）、串口、还是 ROS/ROS 2 接口
4. **控制接口文档**：运动指令、状态回读、急停
5. **视频流地址**（用户提到可以拉流）

安全前提（必须坚持）：

- 先只做**状态回读**（读 IMU/关节角/电量），确认链路通了再说
- 任何**运动指令**都必须有人在场、机器狗架空或垫起、先给最小速度
- 不要在 Jetson 上写的程序里留"上电自动发运动指令"的逻辑

## 四、架构上的建议

```
MID-360 ──以太网直连──> enP8p1s0 (192.168.1.50)   [专线，不设网关]
                              │
                        livox_ros_driver2
                              │
                    /livox/lidar  /livox/imu
                              │
                        FAST-LIO (后续)  ──> 里程计/点云地图
                              │
                        宇泛 SDK (后续)  ──> 运动控制
Wi-Fi (192.168.100.191) ──> 远程 SSH / 调试 / 数据传输   [默认路由走这里]
```

- 有线专线**永不设默认网关**，避免抢 Wi-Fi 的路由（已按此配置）
- 后续如果要加第二台网口设备（比如狗身上的相机），另建独立 profile，同样不设网关

## 五、已知的小坑（留给以后）

1. `colcon version-check` 会联网查 PyPI，网络不好时**看起来像卡死**。别用它判断 colcon 是否存在。
2. GitHub 的 git 协议在这台机器上不稳（`HTTP/2 stream not closed cleanly`）。拉包建议加
   `git -c http.version=HTTP/1.1`，或直接用 codeload tarball。
3. `tmux` / `screen` 都没装，远程跑长任务用 `setsid nohup ... &`；要装的话
   `sudo apt install tmux`（未装，避免动系统）。
4. `livox_ros_driver2` 的 `build.sh` 会 **`rm -rf ../../build ../../devel ../../install`**，
   即清空整个 `~/ws_livox` 的构建产物。不要把别的包放进同一个工作空间，除非接受一起重建。
5. 停止驱动要 `SIGINT` 再补 `TERM`，否则可能残留同名节点。
