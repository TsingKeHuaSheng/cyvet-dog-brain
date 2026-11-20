# 宇泛灵猫 Cyvet 机器狗接入状态（2026-11-12）

## 一、身份与固件

| 项目 | 实测值 | 来源 |
|---|---|---|
| SN | `E03C1CBAAC40A101` | SDK 发现 |
| 型号 | `CYVET-CREATOR` | SDK `deviceModel` |
| deviceCP | `EA0002` | SDK |
| 生产日期 | `20260822` | SDK `productDate` |
| **固件** | **`Cyvet-V1.00.000`** | `system.version` |
| 控制权持有者 | `motionOwner = 0`（无人持有） | `system` |

> **重要**：SDK 标签必须与固件匹配。固件为 `V1.00.000`，因此两个仓库都 checkout
> **`Cyvet-V1.00.000`**（不是文档里原先写的 `V1.00.001`）。

## 二、网络

| 接口 | IP / 状态 | 说明 |
|---|---|---|
| `wlan0` | **`192.168.100.114`** / 网关 `192.168.100.1` / SSID 清科华胜 | **当前在用**，iPad 了 .147 变成了 .114 |
| `eth0` | MAC `e0:3c:1c:ba:ac:40`，link up | **机器狗自带的有线口**（当前未使用） |
| `eth1` (mobile) | `10.112.0.101` / 网关 `10.112.0.99` | 4G 模块，SIM 已插 |

MAC `fc:37:6d:67:ab:5f` 与 RTSP 探测到的主机一致，确认是同一台设备。

> ⚠️ **IP 不固定**：从 `.147` 漂到了 `.114`。建议在路由器上做 DHCP 保留，
> 或让机器狗设静态 IP，否则每次都要重新发现。

## 三、Orin ↔ 机器狗 连通性

- Orin 侧网卡：`wlP1p1s0`（`192.168.100.191`）
- SDK `--iface` **必须填 `wlP1p1s0`**，不能填 `enP8p1s0`（那是雷达专线）
- `--discover-only` 实测发现 1 台机器狗，DDS 初始化正常
- 已确认**没有 AP 客户端隔离**

## 三之二、IP 会变，以及怎么处理（2026-11-12 实测）

### 事实

机器狗的 `wlan0` 地址由路由器 DHCP 分配（网关/DNS 均指向 `192.168.100.1`），
本次观察到它从 **`192.168.100.147` 漂移到了 `192.168.100.114`**。

触发原因可能是租约过期、路由器重启、或原地址被其它设备占用
（同网段实测有 `.1/.103/.105/.106/.108/.112` 等多台设备在抢地址池）。

### 影响面

| 用途 | 是否受 IP 变化影响 | 原因 |
|---|---|---|
| **SDK 运动控制** | ❌ **不受影响** | 官方设计按 **SN** 发现。README 明确：IP 只用于筛选，`--device-id` 必须传 SN，不能传 IP |
| **RTSP 视频** | ✅ **受影响** | RTSP URL 里必须写 IP |

### 解决方案（已实现，不改任何设备配置）

新增两个脚本，实现「按 SN 查当前 IP → 动态拼 RTSP 地址」：

| 脚本 | 作用 |
|---|---|
| `~/get_dog_ip.py` | 通过 SDK discovery 读出当前 `wlan.ipv4Addr`，**stdout 只输出纯 IP**（便于 shell 捕获），详情走 stderr |
| `~/dog_rtsp.sh [channel]` | 内部调用上者，输出完整 RTSP 地址 |

用法：

```bash
source ~/env_uniubi.sh
~/get_dog_ip.py                    # -> 192.168.100.114
~/dog_rtsp.sh 1                    # -> rtsp://<当前IP>:554/live?channel=1&stream=0
curl_rtsp=$(~/dog_rtsp.sh 2)       # 可直接用于 ffplay/gst/OpenCV
```

**已实测**：用动态得到的地址走 `rtspsrc → nvv4l2decoder → appsink` 抓到 1280×720 帧，PASS。

### 更彻底的做法（可选，需路由器权限）

在路由器上按 MAC `fc:37:6d:67:ab:5f` 做 **DHCP 地址保留**。这样地址永久固定，
但并非必需——上面的动态发现已经能覆盖。

> 注意：SDK 的 Python 绑定**没有**修改机器狗网络配置的接口
> （只有 `set_network_interface()`，那是设置**本机**网卡）。
> 因此不建议用脚本去改狗的静态 IP，只能走官方 App。

## 四、SDK 安装

| 项目 | 路径 / 版本 |
|---|---|
| C++ 运行库 | `~/uniubi_robot_sdk`（tag `Cyvet-V1.00.000`，含 `lib/aarch64_host/`） |
| Python 包 | `~/uniubi_robot_sdk_py`（tag `Cyvet-V1.00.000`） |
| venv | `~/venvs/uniubi` |
| 安装方式 | `pip install . -Ccmake.define.PLATFORM=aarch64_host -Cbuild-dir=build/aarch64_host --no-build-isolation` |
| 架构选择 | **`aarch64_host`**（外部 ARM64 主机模式），不是 `aarch64`（大脑本机模式） |
| `MEDIA_ENABLED` | `True` |

### 运行前必须设置的环境变量

```bash
export UNIUBI_SDK_ROOT="$HOME/uniubi_robot_sdk"
export LD_LIBRARY_PATH="$UNIUBI_SDK_ROOT/lib/aarch64_host${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

`lib/aarch64_host/` 里自带 CycloneDDS 0.10.5、iceoryx、libcrypto 等依赖，
所以这个路径必须加进 `LD_LIBRARY_PATH`，不能只装 wheel。

### 环境要求核对（全部满足）

| 要求 | 实际 |
|---|---|
| glibc ≥ 2.34 | 2.35 ✅ |
| `GLIBCXX_3.4.30` | libstdc++.so.6.0.30 提供 ✅ |
| `libatomic.so.1` | 有 ✅ |

### ⚠️ 两个坑

1. **不要在 `~/uniubi_robot_sdk_py` 目录里 import**
   否则 Python 会加载源码目录下的 `robot_motion_sdk/`（没有编译好的 `.so`），
   报 `cannot import name '_uniubi_robot_motion_py_native'`。**请 `cd /tmp` 或任意其它目录再运行。**
2. `Error, cannot connect to unix address /tmp/logServer.sock` 是**无害警告**，
   表示本机没有宇泛的日志服务，不影响功能。

## 五、只读验证结果（未发送任何运动指令）

| 项目 | 结果 |
|---|---|
| `--discover-only` | PASS，发现 1 台 |
| 只读连接 | `[PASS] connected; no action has been started` |
| `system` | 电池 **84%** / 48.43V / 20°C / `motorReady: true` / `motionOwner: 0` |
| `state` | `{}` —— 无进行中动作（合法） |
| `caps` | 取到动作列表与参数范围 |
| `motors` | `motorNum=12`，4 腿 × 3 关节 |

### `caps` 关键内容

| 动作 | 遥控按键 | 参数范围 |
|---|---|---|
| `emergencyStop` | `LB`+`RB` | — |
| `waveHand` | `B` | — |
| `waveBody` | `Start`+`LB` | `axesRY` ±0.244 rad, `lineVelocityX` +0.03/−0.06, `lineVelocityY` ±0.227, `velocity` ±0.349 |
| `tweak` | `A` | — |
| `standing` | `Start` | — |
| **`walking`** | `Back`+`Y` | **`lineVelocityX` ±1.5 m/s，`lineVelocityY` ±1.0 m/s，`velocity` ±2.5 rad/s** |

> 注意：`walking` 的速度上限比之前实验用的 `0.1` m/s 大得多。**首次运动测试务必从小值起。**

### 关节命名

```
leftFrontLeg.hip / .thigh / .calf
rightFrontLeg.hip / .thigh / .calf
leftRearLeg.hip / .thigh / .calf
rightRearLeg.hip / .thigh / .calf
```

## 六、摄像头 / RTSP

两条流：

```
rtsp://192.168.100.114:554/live?channel=1&stream=0
rtsp://192.168.100.114:554/live?channel=2&stream=0
```

| 项目 | 实测 |
|---|---|
| 编码 | H.265 (HEVC), `profile-id=1` |
| 分辨率 | **1280×720** |
| 声明帧率 | 25 fps |
| 音频 | PCMU/8000（存在，本次未用） |
| 封装 | `a=packetization-supported:UNIUBI`（私有标注，但标准 RTSP 客户端可用） |
| 服务器 | `Rtsp Server/3.0` |

### 实测：两路确实是不同相机

- channel=1 与 channel=2 的 SPS/PPS/VPS 字节完全相同（同一编码器配置）
- 但实拍画面**视角错开**，符合双目小基线特征
- 结论：**两路可用，是立体像对**；左右映射仍需贴标记确认

### ⚠️ 会话限制

连续快速发多个 `DESCRIBE` 会返回 **`503 Service Unavailable`**。
间隔几秒、一次只用一个客户端就正常。**不是相机故障。**

### Jetson 上的解码条件

| 组件 | 状态 |
|---|---|
| GStreamer | 1.20.3 |
| `nvv4l2decoder` | ✅ **硬件 H.265 解码可用** |
| `rtph265depay` / `h265parse` | ✅ |
| OpenCV | 4.10.0，FFMPEG + GStreamer 后端均可用 |
| ffmpeg / ffplay | ❌ 未安装（不影响，用 GStreamer 即可） |

可用管线（已验证抓到帧）：

```
rtspsrc location=<URL> protocols=tcp latency=100 !
rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv !
video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR !
appsink drop=true max-buffers=1 sync=false
```

## 七、下一步

1. 确认两路相机的**左右映射**（贴标记拍一张）
2. 规划双目标定（需要棋盘格 + 机器狗静止）
3. 把 YOLO 从 VM 迁到 Orin（硬件解码 + GPU 推理）
4. 之后再考虑雷达-相机外参、FAST-LIO、运动闭环
