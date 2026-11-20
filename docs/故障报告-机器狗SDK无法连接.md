# 故障报告：外部主机无法通过 SDK 连接机器狗（DDS/RPC 无响应）

- 报告日期：2026-11-18
- 设备：宇泛灵猫 Cyvet 四足机器人，型号 `CYVET-CREATOR`，SN `E03C1CBAAC40A101`
- 固件：`Cyvet-V1.00.000`（**未升级过**，用户确认）
- 外部主机：Jetson `nvidia@nvidia-desktop`，Ubuntu 22.04 / JetPack 6.x，`aarch64`
- SDK：C++ `uniubi_robot_sdk` tag `Cyvet-V1.00.000`（`SDK 1.0.0 (commit fb52731b)`）+ Python `uniubi_robot_sdk_py` 同 tag
- 平台参数：`PLATFORM=aarch64_host`（外部 ARM64 主机模式）

---

## 一、结论

**机器狗本体的电源、无线网络、RTSP 视频、遥控器控制全部正常；唯独对外的 DDS/RPC 服务（SDK 侧称 RobotServer / host proxy）在其所在网络上完全不可见。**

即：外部主机（Jetson）与机器狗之间，二层/三层连通性、双向多播、DDS 端口全部验证正常，但机器狗在 **61 个 DDS 域上都没有发出任何 DDS 报文**，导致 SDK 的设备发现与 RPC 调用全部超时。

该状态无法从外部主机侧修复，需要从机器狗侧确认对外服务进程的状态。

---

## 二、现象

### 2.1 设备发现失败（C++ 与 Python 两个 SDK 表现一致）

```bash
$ python3 ~/uniubi_robot_sdk_py/examples/example_highlevel.py --iface wlP1p1s0 --discover-only
[PASS] dds inited with host xml=/tmp/motion_sdk_host_22295.xml, iface=wlP1p1s0
motion sdk platform detected: 1 brain domain=-1,cere domain=42
motion sdk initialized, clientId=uniubi-python-highlevel-cli, multiDevice=1, config=(null)
start new discover window: 5000ms
[WARN] no discovery callback in 5s; retrying once
[WARN] no robots discovered
```

自研 C++ 程序（同样调用 `IMotionSdkService::discoverDevices` / `IMotionHighLevelClient`）结果完全相同。

### 2.2 直连 SN 时 RPC 超时

```
[STEP] connect(lease=60000ms) ...
high level connected: clientId=cyvet-dog-demo leaseHint=60000ms
cere motion rpc robotAppService::getMotionCapabilities failed: timeout
[FAIL] 能力表不可用
```

`connect()` 本地成功，但向狗发起的 `getMotionCapabilities` RPC 超时。

### 2.3 同一台机器狗在 2026-11-12 曾正常工作

历史记录（`~/robot_ws/diagnostics/dog_integration_status.md`）：

| 项目 | 11-12 实测 |
|---|---|
| `--discover-only` | PASS，发现 1 台 |
| 只读连接 | `[PASS] connected; no action has been started` |
| 电池 | 84% / 48.43V / 20°C，`motorReady: true` |

此后固件与 SDK 版本均未变更。

---

## 三、已排除项（附实测数据）

### 3.1 网络连通性：正常

| 检查 | 结果 |
|---|---|
| Jetson 网卡 | `wlP1p1s0` = `192.168.100.191/24` |
| 到狗的路由 | `ip route get 192.168.100.114` → `dev wlP1p1s0 src 192.168.100.191` |
| ICMP | 3 发 3 收，0% 丢包（RTT 254 / 2.07 / 2.69 ms） |
| 狗 RTSP 端口 | `192.168.100.114:554` TCP 可达（从 Jetson 与另一台 Windows 主机双向验证） |
| 狗 MAC | `fc:37:6d:67:ab:5f`（与历史记录一致） |

### 3.2 多播转发：正常（关键证据）

从同网段的另一台 Windows 主机（`192.168.100.100`）向三个多播组注入探针，**Jetson 侧全部收到**：

| 多播组 | Jetson 收到包数 | 来源 |
|---|---|---|
| `239.255.0.1:17900`（DDS 域 42） | 39 | `192.168.100.100` |
| `224.0.0.251:5353`（mDNS） | 40 | `192.168.100.100`、`192.168.100.111` |
| `239.255.255.250:1900`（SSDP） | 41 | `192.168.100.100`、本机 |

说明：AP 未阻断客户端间多播，Jetson 的多播接收链路完好。

### 3.3 DDS 域号不匹配：排除

Jetson 上监听 `239.255.0.1` 组上全部 61 个 DDS 域（端口 `7400 + 250×域号`，域号 0…60），持续 20 秒：

```
已监听 61 个 DDS 域，20 秒...
domain=0   port=7400   来源=['100.114.230.93', '127.0.0.1', '192.168.1.50', '192.168.100.191']
--- 扫描结束 ---
```

唯一有流量的域 0，其 4 个来源全部是 **Jetson 自身的地址**（tailscale / 回环 / 雷达网口 / Wi-Fi），属于本机某进程的自身广播。

**全部 61 个域中，没有任何一个出现 `192.168.100.114` 的报文。**

### 3.4 机器狗端口扫描

| 端口 | 状态 |
|---|---|
| 554 (RTSP) | OPEN |
| 443 | TCP 可连，但 TLS 握手失败（`curl exit 35`），非标准 HTTPS |
| 21 / 22 / 23 / 80 / 1883 / 5555 / 8000 / 8080 / 8081 / 8883 / 9090 / 10000 | 全部 CLOSED |

无 22，无法登录机器狗内部检查服务状态。

### 3.5 机器狗本体：正常

- 遥控器可正常控制机器狗行走（用户实测确认）
- RTSP 两路视频可正常取流
- 因此电源、运控小脑、无线链路、图传服务均正常

---

## 四、判断

机器狗上的 **RTSP 服务**与 **遥控器控制链路**均独立于 SDK 所依赖的对外 RPC 服务。当前状态符合以下任一情况：

1. 对外 RPC 服务（RobotServer / host proxy）进程未启动、已崩溃，或未随系统启动；
2. 该服务依赖的"开发者模式 / 开放接口"开关处于关闭状态；
3. 该服务仅在特定条件下对外提供（例如遥控器断开后、或非充电状态）；
4. 该服务绑定的网络接口 / DDS 域配置与本 SDK 预期不一致。

外部主机侧无法区分以上四种情况，需要设备侧信息。

---

## 五、请求协助确认的事项

1. **远程检查机器狗上对外 RPC 服务的进程状态**：是否存在、是否在运行、启动日志有无异常报错。
2. 确认该服务是否受 App 中的"开发者模式 / SDK 调试 / 开放接口"开关控制，当前状态如何。
3. 确认该服务对外提供 RPC 是否有前置条件（遥控器连接状态、充电状态、运控模式等）。
4. 如服务异常，请提供：在设备上重启该服务的方法，或需要执行的固件修复/升级步骤。
5. 如判断需要升级固件，请确认目标版本与应使用的 SDK tag（当前 SDK 为 `Cyvet-V1.00.000`）。

---

## 六、复现命令（供贵方参考）

外部主机（Jetson）上执行：

```bash
source ~/env_uniubi.sh

# 1) 网络
ip -br addr
ip route get 192.168.100.114
ping -c 3 192.168.100.114

# 2) SDK 发现（C++）
cd ~/cyvet_demo && ./run.sh --discover

# 3) SDK 发现（Python，官方示例）
python3 ~/uniubi_robot_sdk_py/examples/example_highlevel.py \
  --iface wlP1p1s0 --discover-only

# 4) 直连 SN 的只读 RPC（预期在 getMotionCapabilities 处 timeout）
cd ~/cyvet_demo && ./run.sh --read-only
```

---

## 七、附：环境对照

| 项目 | 值 |
|---|---|
| 机器狗 SN | `E03C1CBAAC40A101` |
| 机器狗 IP | `192.168.100.114`（DHCP，曾为 `.147`） |
| 机器狗 MAC | `fc:37:6d:67:ab:5f` |
| 机器狗固件 | `Cyvet-V1.00.000`（未变更） |
| Jetson | `nvidia@nvidia-desktop`，`wlP1p1s0` = `192.168.100.191` |
| SDK | `1.0.0 (commit fb52731b)`，tag `Cyvet-V1.00.000`，`PLATFORM=aarch64_host` |
| DDS 域 | cere domain = 42（SDK 日志）；已扫描 0–60 全部域 |
| Wi-Fi | SSID `清科华胜`，网关 `192.168.100.1` |
