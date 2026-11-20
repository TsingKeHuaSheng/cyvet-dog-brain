# 从零到建图：完整操作过程

> 这份文档记录**每一步实际执行的命令**和**为什么要这么做**。
> 没有脚本，全部手动敲，目的是能看懂整个过程。

---

# 第一部分：拿到代码

## 1.1 先搞清楚要拿什么

FAST-LIO 原版（hku-mars/FAST_LIO）只支持 ROS 1。我们要的是 ROS 2 版本，
而且希望它带**回环检测**。所以选了：

```
https://github.com/liangheming/FASTLIO2_ROS2
```

这个仓库里有 5 个包：

| 目录 | 作用 |
|---|---|
| `fastlio2` | 核心：激光惯性里程计 |
| `pgo` | 回环检测 + 位姿图优化 |
| `localizer` | 重定位（以后导航用） |
| `hba` | 地图精修 |
| `interface` | 自定义服务定义（保存地图用） |

## 1.2 实际用的命令

```bash
mkdir -p ~/ws_slam/src
cd ~/ws_slam/src

curl -sSL --http1.1 -o /tmp/fastlio2.tar.gz \
  https://codeload.github.com/liangheming/FASTLIO2_ROS2/tar.gz/refs/heads/main

tar xzf /tmp/fastlio2.tar.gz
mv FASTLIO2_ROS2-main FASTLIO2_ROS2
```

**逐条解释：**

| 命令 | 干什么 | 为什么这么写 |
|---|---|---|
| `mkdir -p ~/ws_slam/src` | 建工作空间 | ROS 2 约定用 `src/` 放源码 |
| `curl -o` | 下载压缩包 | 见下方为什么不用 git clone |
| `--http1.1` | 强制 HTTP/1.1 | 这台机器用 HTTP/2 拉 GitHub 经常断 |
| `tar.gz/refs/heads/main` | codeload 的地址格式 | 直接下载分支压缩包，不走 git 协议 |
| `tar xzf` | 解压 | x=解压 z=gzip f=文件 |
| `mv ...-main` | 改目录名 | 解压出来叫 `FASTLIO2_ROS2-main`，去掉后缀 |

## 1.3 为什么不用 git clone

试过，会报错：

```
error: RPC failed; HTTP/2 stream 1 was not closed cleanly
fatal: the remote end hung up unexpectedly
```

这台机器到 GitHub 的 git 协议经常被重置。codeload 是另一条路（HTTP 直接下压缩包），
稳定得多。

**如果以后网络好了，用 git clone 更好**（能切版本、能 pull 更新）：

```bash
git clone https://github.com/liangheming/FASTLIO2_ROS2.git
```

## 1.4 验证拿到的代码

```bash
ls ~/ws_slam/src/FASTLIO2_ROS2
```

应该看到：

```
README.md  fastlio2  hba  interface  localizer  pgo
```

---

# 第二部分：装依赖

## 2.1 先查缺什么

```bash
for p in libeigen3-dev libpcl-dev ros-humble-pcl-ros \
         ros-humble-pcl-conversions ros-humble-tf2-ros ros-humble-gtsam; do
  dpkg -s $p >/dev/null 2>&1 && echo "有 $p" || echo "缺 $p"
done
```

## 2.2 装系统包

```bash
sudo apt-get install -y libeigen3-dev libpcl-dev ros-humble-pcl-ros \
                        ros-humble-pcl-conversions ros-humble-tf2-ros \
                        ros-humble-gtsam libsuitesparse-dev
```

| 包 | 作用 |
|---|---|
| `libeigen3-dev` | 矩阵运算（FAST-LIO 全靠它） |
| `libpcl-dev` | 点云库，读写 .pcd |
| `ros-humble-pcl-ros` | ROS 2 与 PCL 的桥接 |
| `ros-humble-tf2-ros` | 坐标变换 |
| `ros-humble-gtsam` | 回环优化求解器（**apt 里就有，不用源码编译**） |
| `libsuitesparse-dev` | GTSAM 的依赖 |

## 2.3 装 Sophus（重点：别编译）

FAST-LIO 源码里有这一行：

```cpp
#include <sophus/so3.hpp>
```

所以需要 Sophus 的头文件。

### 常规做法（慢，不推荐）

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j4          # ← 这里会卡 10 分钟！
sudo make install
```

**为什么慢**：Sophus 的 CMake 默认会编译**一堆测试用例**（test_cartesian3、
test_sim3 之类），那些我们根本不需要。在 Orin Nano 4 核上要 10 分钟以上。

### 实际做法（5 秒）

Sophus 是**纯头文件库**——只有 `.hpp`，没有需要编译的 `.cpp`。所以：

```bash
cd /tmp
curl -sSL -o sophus.tar.gz \
  https://codeload.github.com/strasdat/Sophus/tar.gz/refs/tags/1.22.10
tar xzf sophus.tar.gz
sudo cp -r Sophus-1.22.10/sophus /usr/local/include/
```

**为什么这样就行**：`/usr/local/include` 是编译器的默认搜索路径，
`#include <sophus/so3.hpp>` 自动能找到。

验证：
```bash
ls /usr/local/include/sophus/so3.hpp
```

---

# 第三部分：编译

## 3.1 关键：先 source 两个环境

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash     # ← 这行最容易忘
```

**为什么必须 source 雷达工作空间**：
`fastlio2` 的 `CMakeLists.txt` 里有：

```cmake
find_package(livox_ros_driver2 REQUIRED)
```

`colcon` 靠环境变量 `AMENT_PREFIX_PATH` 找包。不 source 就找不到，编译直接失败。

## 3.2 编译

```bash
cd ~/ws_slam
colcon build --symlink-install
```

**`--symlink-install` 是什么**：安装时不复制文件，而是建符号链接指向 `src/`。
好处是改了源码不用重新 build。开发阶段强烈建议加。

**第一次编译约 5 分钟**（4 核）。中途会有很多 `-Wsign-compare` 警告，**不用管**，
那不是错误。

成功的标志：

```
Summary: 5 packages finished [5min 12s]
```

## 3.3 验证

```bash
source ~/ws_slam/install/setup.bash
ros2 pkg list | grep -E 'fastlio2|pgo|localizer|hba|interface'
```

---

# 第四部分：启动雷达

## 4.1 两种模式，别搞混

| launch 文件 | 输出类型 | 谁用 |
|---|---|---|
| `msg_MID360s_launch.py` | `livox_ros_driver2/msg/CustomMsg` | **FAST-LIO 建图** |
| `msg_MID360s_pc2_launch.py` | `sensor_msgs/msg/PointCloud2` | RViz 实时看点云 |

**区别在参数 `xfer_format`**：`1` = CustomMsg（紧凑，给算法用），
`0` = PointCloud2（标准，给可视化用）。

**RViz 只能显示 PointCloud2，显示不了 CustomMsg。**

## 4.2 启动（建图用 CustomMsg）

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

**期望看到的关键日志：**

```
[INFO] [livox_lidar_publisher]: Init lds lidar success!
[INFO] [livox_lidar_publisher]: successfully set lidar attitude, ip: 192.168.1.194
[INFO] [livox_lidar_publisher]: successfully change work mode
[INFO] [livox_lidar_publisher]: livox/imu publish use imu format
[INFO] [livox_lidar_publisher]: livox/lidar publish use livox custom format
```

**如果卡在 `Init lds lidar success!` 后面没了** = 驱动在跑但没认出雷达。
查供电、网线、`192.168.1.50` 这个地址。

## 4.3 验证数据

**另开一个终端：**

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

ros2 topic hz /livox/lidar     # 应该约 10 Hz
ros2 topic hz /livox/imu       # 应该约 200 Hz
```

---

# 第五部分：启动 FAST-LIO

## 5.1 手动起，别用 launch（这样看得清）

**再开一个终端：**

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_slam/install/setup.bash

ros2 run fastlio2 lio_node --ros-args \
  -r __ns:=/fastlio2 \
  -p config_path:=$(ros2 pkg prefix fastlio2)/share/fastlio2/config/lio.yaml
```

**逐段解释：**

| 片段 | 含义 |
|---|---|
| `ros2 run fastlio2 lio_node` | 跑 `fastlio2` 包里的 `lio_node` 程序 |
| `--ros-args` | 后面是 ROS 参数 |
| `-r __ns:=/fastlio2` | 命名空间设为 `/fastlio2`，话题变成 `/fastlio2/xxx` |
| `-p config_path:=...` | 传参数：配置文件路径 |
| `$(ros2 pkg prefix fastlio2)` | 动态获取包安装路径（不写死） |

**期望日志：**

```
[INFO] [lio_node]: LIO Node Started
[INFO] [lio_node]: LOAD FROM YAML CONFIG PATH: .../lio.yaml
Multi thread started
```

## 5.2 起回环节点（第三个终端）

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_slam/install/setup.bash

ros2 run pgo pgo_node --ros-args \
  -r __ns:=/pgo \
  -p config_path:=$(ros2 pkg prefix pgo)/share/pgo/config/pgo.yaml
```

期望：`PGO node started`

## 5.3 查看输出

```bash
ros2 topic list | grep -E 'fastlio2|pgo'
```

应该看到：

```
/fastlio2/body_cloud      机体系点云
/fastlio2/lio_odom        位姿（10 Hz）
/fastlio2/lio_path        轨迹
/fastlio2/world_cloud     世界系点云（这就是地图）
/pgo/loop_markers         回环连线
```

看频率：
```bash
ros2 topic hz /fastlio2/lio_odom    # 约 10 Hz
```

看位姿（静止时应该接近 0）：
```bash
ros2 topic echo --once /fastlio2/lio_odom
```

---

# 第六部分：RViz 看实时建图

## 6.1 启动 RViz

```bash
source /opt/ros/humble/setup.bash
rviz2
```

## 6.2 三步配置

**第一步**：左上角 `Global Options` → `Fixed Frame` 改成 **`map`**

**第二步**：左下角 `Add` → 选 `PointCloud2` → OK

**第三步**：新加的显示点开，`Topic` 填 **`/fastlio2/world_cloud`**

## 6.3 还能加的显示

| 想看的 | Topic | 显示类型 |
|---|---|---|
| 实时点云 | `/fastlio2/world_cloud` | PointCloud2 |
| 轨迹 | `/fastlio2/lio_path` | Path |
| 回环连线 | `/pgo/loop_markers` | Marker |

## 6.4 怎么判断建得好不好

| 现象 | 含义 |
|---|---|
| 墙是一条细线 | ✅ 正常 |
| 墙变成一堵厚带 | ❌ 有漂移 |
| 同一面墙出现两层 | ❌ 严重漂移/重影 |
| 走回起点后轨迹首尾靠拢 | ✅ 回环触发了 |

---

# 第七部分：保存地图

## 7.1 先建目录（服务不会自动建）

```bash
mkdir -p ~/robot_ws/maps/run1
```

**这一步不做会报**：`... IS NOT EXISTS!`

## 7.2 调用保存服务

**新开一个终端：**

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_slam/install/setup.bash

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/nvidia/robot_ws/maps/run1', save_patches: true}"
```

**参数解释：**

| 参数 | 含义 |
|---|---|
| `file_path` | 保存目录（必须已存在） |
| `save_patches` | 是否同时存子图（回环优化需要，建议 true） |

**期望返回：**

```
response:
interface.srv.SaveMaps_Response(success=True, message='SAVE SUCCESS!')
```

## 7.3 看产出

```bash
ls -la ~/robot_ws/maps/run1/
```

```
map.pcd        完整地图
poses.txt      每个关键帧的位姿（文件名 x y z qw qx qy qz）
patches/       子图
```

## 7.4 ⚠️ 顺序不能反

**保存必须在 Ctrl+C 之前。** 程序一停，内存里的地图就没了。

---

# 第八部分：看已保存的地图

## 8.1 发布 pcd 文件

```bash
source /opt/ros/humble/setup.bash

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:=/home/nvidia/robot_ws/maps/run1/map.pcd \
  -p tf_frame:=map
```

**注意：文件路径是 `-p` 参数，不是位置参数。**

写错了会报：
```
Statically typed parameter 'file_name' must be initialized
```

**`tf_frame:=map`** 是把点云的坐标系标成 `map`，这样 RViz 里 Fixed Frame 选 `map`
才能看到。

## 8.2 RViz 里看

```bash
rviz2
```

1. `Fixed Frame` 选 **`map`**
2. `Add` → `PointCloud2` → `Topic` 填 **`/cloud_pcd`**

## 8.3 小知识：只发一次也能看到

`pcd_to_pointcloud` 只发布一次，但用的是"锁存"（transient local）方式，
后连上来的订阅者也能收到最后一条。所以先开发布还是先开 RViz 都行。

---

# 第九部分：判断质量

## 9.1 看一个数：闭合误差

```bash
python3 - <<'PY'
import numpy as np
S = np.array([[float(v) for v in l.split()[1:4]]
              for l in open('/home/nvidia/robot_ws/maps/run1/poses.txt')])
print("关键帧:", len(S))
print("闭合误差: %.3f m" % np.linalg.norm(S[-1]-S[0]))
PY
```

**合格线：< 0.3 m**

## 9.2 看三个图

俯视图、侧视图、墙带图——需要把 pcd 画出来。这部分要写一点 Python
（用 numpy 读 pcd，用 PIL 画图），跟主流程无关，属于"事后分析"。

---

# 附：常见错误对照表

| 报错/现象 | 原因 | 解决 |
|---|---|---|
| `find_package(livox_ros_driver2) failed` | 没 source 雷达工作空间 | `source ~/ws_livox/install/setup.bash` |
| RViz 里点云不动/空白 | Fixed Frame 填错 | 填 `livox_frame` 或 `map` |
| RViz 收不到 CustomMsg | 启动的是 msg 版驱动 | 换用 pc2 版 launch |
| `IS NOT EXISTS!` | 目录没建 | `mkdir -p` |
| `file_name must be initialized` | pcd 路径写法错 | 用 `-p file_name:=` |
| `process has died ... rviz2` | SSH 无图形界面 | 在桌面终端跑，或 `DISPLAY=:0` |
| 地图是一团球 | 雷达没动 | 必须边走边建 |
| 回环不触发 | 没回到起点 1 m 内 | 走闭合路线 |
| 端口 56301 冲突 | 起了两个驱动 | 先 `pkill -f livox_ros_driver2_node` |
