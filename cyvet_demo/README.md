# 用 UniUbi SDK 写程序、编译、运行、驱动机器狗

**机器狗**：宇泛灵猫 Cyvet（SN `E03C1CBAAC40A101`）
**SDK**：`uniubi_robot_sdk`（C++）/ `uniubi_robot_sdk_py`（Python）/ `uniubi_ros2`（ROS 2）
**本文的编译机**：Jetson `192.168.100.191`（`nvidia@nvidia-desktop`），外部 ARM64 主机
**控制方式**：远端 High-level（SDK → RobotServer RPC → MotionServer RPC）

---

## 0. 三条命令（最短路径）

在 **Jetson** 上（不是这台 Windows）：

```bash
cd ~/cyvet_demo
./build.sh                                              # 编译
./run.sh --discover                                     # 确认狗在线、拿到 SN
./run.sh --read-only                                    # 只读预检，不运动
./run.sh --yes                                          # 站立 → 前进1m → 左转 → 趴下
```

---

## 1. 先搞清楚三件事，否则一定卡住

### 1.1 在哪编译 —— 架构由"程序最终在哪台机器跑"决定

SDK 是按架构交付预编译运行库的，头文件和 `lib/<arch>/` 必须同版本同架构：

| 程序跑在哪 | 架构目录 | 编译参数 | 能用什么 |
|---|---|---|---|
| 狗的大脑板（Orin） | `lib/aarch64/` | 默认 | High-level + **Low-level** + MediaBus |
| 外部 ARM64 主机（本项目的 Jetson） | `lib/aarch64_host/` | **`-DPLATFORM=aarch64_host`** | 远端 High-level、音频 |
| x86_64 Linux 主机 | `lib/x86_64/` | 默认 | 远端 High-level、音频 |

> **最容易踩的坑**：Jetson 是 ARM64，`CMAKE_SYSTEM_PROCESSOR=aarch64`，默认会被当成狗的大脑板，去链接 `lib/aarch64/`（那是 Orin 上带 CUDA 的库，跑不了）。必须显式 `-DPLATFORM=aarch64_host`。

这台 Windows 编不了 —— 运行库是 Linux/ARM64 的 ELF 动态库，Windows 上既没有对应的 g++，也没有能加载它们的系统。所以"写 → 编 → 跑"都在 Jetson 上做。

### 1.2 连谁 —— 网卡 + SN，两个都必须显式给

```
   ┌──────────────┐   Wi-Fi 192.168.100.0/24    ┌─────────────────┐
   │ Jetson       │ ──────────────────────────► │ 机器狗 Cyvet    │
   │ .100.191     │   网卡 wlP1p1s0             │ 小脑运控 + 大脑 │
   │ (编译/运行)   │                             │ SN=E03C1CBAAC40A101 │
   └──────────────┘                             └─────────────────┘
        ▲ 你的 Windows 也在同一网段（192.168.100.100）
```

- **网卡**：必须是能到达狗的那块。Jetson 上是 `wlP1p1s0`（它的 Wi-Fi）。
- **设备**：只认 **SN**，不能传 IP。IP 会变，SN 不变。
- 外部主机即使只控制一台狗，也必须同时给网卡和 SN —— SDK 不会"自动选发现到的第一台"。

### 1.3 权限 —— 遥控器必须先断开

High-level 想拿控制权（`startControl`）时，**遥控器必须处于断开状态**：

> 长按遥控器 `M` 键，直到听到语音 **"遥控器连接已断开"**。

遥控器还连着，取权就会被拒（错误码 `kRpcAcquireRejected`）。只读查询（`--read-only`）不受这条限制。

急停/接管：再按 `M` 键直到 **"遥控器已连接"**，然后用遥控器接管。

---

## 2. 现场准备：4 条检查命令

```bash
# ① 看哪块网卡在狗的网段（找有 192.168.100.x 地址的那块）
ip -br addr

# ② 环境变量：SDK 根目录 + 运行库路径
source ~/env_uniubi.sh
echo "$UNIUBI_SDK_ROOT  $UNIUBI_IFACE  $UNIUBI_DOG_SN"

# ③ 确认运行库目录是 aarch64_host（外部主机）
ls "$UNIUBI_SDK_ROOT/lib/"

# ④ 确认狗在线（会打印 SN 和 info JSON，含它当前 IP）
python3 ~/get_dog_ip.py --iface wlP1p1s0 --sn E03C1CBAAC40A101 --timeout 20
```

`get_dog_ip.py` 打印的纯 IP 就来自 SDK 的发现结果，能拿到 IP 就说明 SDK 链路是通的。

---

## 3. 把代码放上去

本工程在 `D:\机器狗项目清科华胜\cyvet_demo\`，三个文件：

```
cyvet_demo/
├── src/cyvet_dog_demo.cpp    # 主程序（约 480 行，带中文注释）
├── CMakeLists.txt            # 独立业务工程，不依赖 SDK 源码树
├── build.sh                  # 编译
└── run.sh                    # 运行（自动带 LD_LIBRARY_PATH / 网卡 / SN）
```

从 Windows 拷到 Jetson（在 PowerShell 里执行）：

```powershell
scp -r "D:\机器狗项目清科华胜\cyvet_demo" nvidia@192.168.100.191:~/cyvet_demo
```

拷完在 Jetson 上补一下可执行位和行尾：

```bash
cd ~/cyvet_demo
chmod +x build.sh run.sh
sed -i 's/\r$//' build.sh run.sh          # 万一行尾被 Windows 改过
```

没有 scp 就直接在 Jetson 上用 `nano`/`gedit` 新建同名文件，把内容贴进去。

---

## 4. 编译

```bash
cd ~/cyvet_demo
./build.sh
```

`build.sh` 实际执行的就是这两步（关键是 `-DPLATFORM=aarch64_host`）：

```bash
source ~/env_uniubi.sh
cmake -S . -B build -DPLATFORM=aarch64_host -DUNIUBI_SDK_ROOT="$UNIUBI_SDK_ROOT"
cmake --build build -j"$(nproc)"
```

产物：`~/cyvet_demo/build/cyvet_dog_demo`

编译期要求（Jetson 都满足）：Linux、glibc ≥ 2.34（Ubuntu 22.04 / JetPack 6 是 2.35）、g++ ≥ 9、CMake ≥ 3.18。

验证产物：

```bash
file build/cyvet_dog_demo        # 应为 ELF 64-bit LSB, ARM aarch64
ldd  build/cyvet_dog_demo | grep -i robotMotionSdk   # 应指向 lib/aarch64_host/
```

> CMakeLists 里给产物写入了 `RUNPATH`，所以正常情况不带 `LD_LIBRARY_PATH` 也能跑；`run.sh` 里仍然 export 一次，双保险。

---

## 5. 运行：一定要按这个顺序，别跳步

### 第 1 步：只发现，不连接（零风险）

```bash
./run.sh --discover
```

期望输出：

```
=== Cyvet 机器狗 SDK 示例 ===
[PASS] SDK 初始化完成，版本 1.0.0 (commit xxxxxxx)
[INFO] 网卡=wlP1p1s0
[STEP] 设备发现（只读，5 秒窗口）...
  发现 SN=E03C1CBAAC40A101 info={"version":"Cyvet-V1.00.001", ... "network":{...,"wlan":{"ipv4Addr":"192.168.100.114"}}}
[PASS] 发现设备，用上面的 SN 做 --device-id
```

拿到 SN 和 `version` 后确认两件事：**SN 和你要控的那台一致**；**固件版本和 SDK 版本匹配**（`V1.00.000/.001` 对应 `~/uniubi_robot_sdk`，`sdk_test_v100001` 那套对应新固件）。

### 第 2 步：只读预检（零运动，不需要断遥控器）

```bash
./run.sh --read-only
```

会打印电量/健康度/电压、系统状态 JSON（含网络）、当前活动动作、能力表 JSON（支持哪些动作、每个参数的 min/max/单位）。

这一步跑通 = SDK 链路、鉴权、RPC 全都正常，剩下的只是发动作。

### 第 3 步：真动作（**先把狗放到空旷地面，人站在急停够得着的地方**）

```bash
# 遥控器先断开（长按 M 键，"遥控器连接已断开"）
./run.sh --yes
```

程序流程（每一步都有哨兵输出，任一步失败立刻走安全收尾）：

```
5 秒倒计时（Ctrl-C 可取消）
  ↓
[STEP] 申请 High-level 控制权        startControl(10000)
[PASS] 控制权已拿到（此时还没有任何动作在跑）
  ↓
站立 2 秒                            standUp()
  ↓
前进 0.25 m/s × 4 s（≈1 米）         move(vx, 0, 0)
清零速度                             setActionParams({"lineVelocityX":0,...})
  ↓
原地转向 0.4 rad/s × 2 s（≈46°）     move(0, 0, vyaw)
清零速度                             setActionParams({...})
  ↓
停止动作                             stopAction()
趴下 2 秒                            lieDown()
  ↓
释放控制权 → 断开                     releaseControl() → disconnect()
```

只往前挪一点点，先验证方向对不对：

```bash
./run.sh --yes --no-turn --vx 0.1 --walk-seconds 1
```

---

## 6. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--device-id SN` | 从 `$UNIUBI_DOG_SN` 取 | 目标机器狗 SN，**不是 IP** |
| `--iface IFACE` | `wlP1p1s0` | 能到达狗的网卡 |
| `--discover` | — | 只发现设备后退出 |
| `--read-only` | — | 只做只读预检，不取权不运动 |
| `--vx MPS` | `0.25` | 前进线速度 m/s |
| `--walk-seconds S` | `4.0` | 前进持续秒数 |
| `--vyaw RPS` | `0.4` | 转向角速度 rad/s |
| `--turn-seconds S` | `2.0` | 转向持续秒数 |
| `--no-walk` / `--no-turn` | — | 跳过前进 / 转向 |
| `--stand-seconds S` | `2.0` | 站定后等待时间 |
| `--lease-ms MS` | `60000` | 控制权租约（SDK 内部自动续约） |
| `--yes` | — | 跳过 5 秒倒计时 |
| `--help` | — | 完整帮助 |

速度的**真实上下限和单位以 `--read-only` 打印的能力表为准**（`params` 里每个参数都有 `min`/`max`/`unit`）。不同固件版本可能不一样。

---

## 7. 代码里每一步对应哪个 SDK 接口

| 代码位置 | SDK 接口 | 作用 |
|---|---|---|
| `initService()` | `IMotionSdkService::instance()` | 全局单例入口 |
| 同上 | `setDiscoverCallback()` | 注册发现回调（**必须在 init 之前**） |
| 同上 | `setNetworkInterface(iface)` | 指定 DDS 网卡（**必须在 init 之前**） |
| 同上 | `initialService(nullptr, clientId)` | 初始化共享资源；`nullptr` 走默认配置 |
| `runDiscovery()` | `discoverDevices(5000)` | 只读发现，不连接、不取权 |
| `run()` | `IMotionHighLevelClient::create(sn)` | 按 SN 建远端 client |
| `connect()` | `connect(leaseMs)` | 进高级模式，拿到"已连接"状态 |
| 同上 | `getMotionCapabilities(json)` | 能力表就绪 = RPC 通道真的通了 |
| `preflight()` | `getPowerInfo(&power, 5000)` | 电量/健康度/电压 |
| 同上 | `querySystemStatus(json)` | 系统状态（含网络） |
| 同上 | `queryMotionState(json)` | 当前动作；`{}` = 没有活动动作 |
| `takeControl()` | `startControl(10000)` + `getState()==kControlled` | 异步取权，轮询状态 |
| `runMotionScript()` | `standUp()` / `move(vx,vy,vyaw)` / `setActionParams(json)` / `stopAction()` / `lieDown()` | 动作与调参 |
| `cleanup()` | `releaseControl()` / `disconnect()` | 交还控制权 |
| `main()` 结尾 | `IMotionSdkService::shutdown()` | 释放 SDK 共享资源 |

**两个容易误解的点**（SDK 文档里专门强调过）：

1. `startControl()` 是**异步**的：返回 `true` 只代表"请求已受理"，真正拿到控制权要看 `getState() == kControlled` 或 `ConnectCallback`。程序里用 `waitForState` 等了 10 秒。
2. `startAction("walking")` **不带速度参数时只切姿态、不会走**。要动必须给 `velocity` / `lineVelocityX` / `lineVelocityY`。而且**不需要周期性重发**——一次设置持续生效，直到被覆盖或 `stopAction()`。

---

## 8. 不写代码：直接用 SDK 自带的 CLI 指令

`example_highlevel` 就是 SDK 的官方命令行，编译好之后可以交互式发指令 —— 这就是所谓的"用 SDK 里的指令驱动机器狗"：

```bash
# 编译官方示例（一次就够）
cd ~/uniubi_robot_sdk
cmake -S . -B build-aarch64-host -DPLATFORM=aarch64_host
cmake --build build-aarch64-host -j"$(nproc)" --target example_highlevel

# 运行
source ~/env_uniubi.sh
export LD_LIBRARY_PATH="$UNIUBI_SDK_ROOT/lib/aarch64_host:$LD_LIBRARY_PATH"
./build-aarch64-host/examples/example_highlevel --iface wlP1p1s0 --device-id E03C1CBAAC40A101
```

进入 `highlevel>` 提示符后：

```
status                       # 能力表 + 系统状态 + 当前动作 + 传感器
caps                         # 只打能力表
motors                       # 12 关节布局
odom 5                       # 里程计（只有在走的时候有效）
take                         # 取控制权（遥控器要先断开）
start walking                # 切到 walking 姿态，速度 0，不会走
send 3 {"lineVelocityX":0.3,"lineVelocityY":0,"velocity":0}   # 走 3 秒后自动清零
zero                         # 只清速度，动作不结束
stop                         # 停止当前动作
estop                        # 急停
release                      # 交还控制权
quit                         # 退出
```

> 注意：官方 CLI 退出时**不会自动 `stopAction()`**。如果动作还在跑，先 `stop` 再 `release`，否则动作可能留在服务端。

**不编译也能跑的一条路（Python）**：环境里已经有现成的 venv 和控制台脚本，直接

```bash
~/dog_console.sh                 # 交互控制台
~/dog_console.sh --discover-only # 只发现
```

它是 `~/venvs/uniubi/bin/python` + `uniubi_robot_sdk_py/examples/example_highlevel.py`。注意脚本里 `cd /tmp`，是为了避免 Python 从源码目录导入未编译的 `robot_motion_sdk`。

---

## 9. 安全红线（实机前逐条确认）

1. **场地空旷、平整、无障碍**；第一次只在 1 米尺度内试探。
2. **急停随时可触达**，全程**有人值守**。
3. 遥控器断开才能取权；出问题立刻按 `M` 键让遥控器重新连上接管。
4. **Low-level（关节级）必须先吊起来四脚腾空**——本示例是 High-level，不涉及，但如果你后面改去跑 `example_lowlevel`，这条是硬要求。
5. 本程序的每个动作都有秒数上限；`Ctrl-C` 会走"清速度 → 停止动作 → 释放控制权"的安全收尾，狗保持站立由你接管。
6. 电量低于 30% 会告警：步态类动作电流大，低电量容易触发保护。

---

## 10. 排错

| 现象 | 原因 | 处理 |
|---|---|---|
| `SDK 初始化失败` | `LD_LIBRARY_PATH` 没指向运行库 | `source ~/env_uniubi.sh`；确认 `ls $UNIUBI_SDK_ROOT/lib/aarch64_host` 有 `librobotMotionSdk.so` |
| `没发现设备` | 网卡选错 / 没连狗 Wi-Fi / 狗没开机 | `ip -br addr` 找 `192.168.100.x` 的网卡；`ping` 狗 |
| `kRpcConnectFailed` | SDK 未初始化或通道没起来 | 先 `--discover` 验证链路；确认网卡和狗在同一网段 |
| `kRpcAcquireRejected` | **遥控器还连着**，或控制权被别人占着 | 长按 `M` 键到"遥控器连接已断开"；关掉其它正在控制的程序 |
| `kNotControlled` | 还没拿到控制权就发动作 | 保证 `take`/`startControl` 成功后再发 |
| `kActionRejected` | 动作名不对，或当前姿态不允许该动作 | `caps` 看支持的动作名；先 `standUp` 再 `move` |
| `kInvalidParam` | 参数 JSON 写错，或超出 min/max | 按能力表里的 `min`/`max`/`unit` 给值 |
| `kSessionExpired` | 租约到期被收回 | 调大 `--lease-ms`（SDK 内部会续约，长时间不动可能被服务端回收） |
| 编译报 `libmediaBus.so: no rule to make target` 之类 | 链接期找不到 SDK 同级库 | 确认 `-DPLATFORM=aarch64_host` 且 `ls $UNIUBI_SDK_ROOT/lib/aarch64_host` 库齐全 |
| 编译报 `robotservice/... .h: No such file` | 头文件路径没传 | 检查 `UNIUBI_SDK_ROOT/include/uniubi` 是否存在 |
| 编译通过了但跑起来 `cannot open shared object` | 库架构不对（混了 `lib/aarch64`） | `ldd` 看指向哪个目录，重编时确实带了 `PLATFORM=aarch64_host` |
| 狗站起来了但不走 | `startAction("walking")` 没带速度参数 | 用 `move()` 或 `setActionParams` 给 `lineVelocityX` |

错误码在程序里都会被翻成中文提示，直接看 `[FAIL]` 那行即可。

---

## 11. 下一步能往哪扩

| 想做的事 | 入口 | 备注 |
|---|---|---|
| 摄像头画面 | `rtsp://<狗IP>:554/live?channel=1&stream=0` | 不用 SDK、不用取权，VLC/ffplay 直接看 |
| 订阅音视频帧 | `IMotionHighLevelClient::createMediaBusClient()` | 远端只支持 PCM 采集和 RawBack 播放 |
| 看遥控器按键 | `setObservedEnable({"motionEnable":true,"trcEnable":true})` | 回调里读 `obs.trc`，先判 `valid` |
| 关节级控制（自己跑策略） | `MotionLowLevelClient` | **只能在狗的大脑板上跑**，且必须吊起来四脚腾空 |
| ONNX 策略行走（TensorRT） | `example_lowlevel_tensorrt` | 同样必须在 Orin 上，JetPack 6.2.1 + TensorRT 10.3 |
| 用 ROS 2 集成 | `~/uniubi_ros2`（`uniubi_ros2/launch`、`src`） | DDS 协议直连属高级路径 |

---

## 附：本工程与 SDK 的版本对应

- `~/uniubi_robot_sdk` + `~/uniubi_robot_sdk_py` → 对应固件 **Cyvet-V1.00.000**
- `~/sdk_test_v100001/uniubi_robot_sdk{,_py}` + `~/venvs/uniubi_v100001` → 对应固件 **Cyvet-V1.00.001**

用 `--read-only`（或 `--discover` 打印的 `version`）确认狗的实际固件版本，**不要跨版本/跨架构混用运行库和头文件**。切换 SDK 只需：

```bash
source ~/env_uniubi_v100001.sh     # 换到新版本
./build.sh                          # 用新版本重编（务必删掉 build/ 重新配置）
```
