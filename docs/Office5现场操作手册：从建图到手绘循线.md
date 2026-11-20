# Office5 现场操作手册：从建图到手绘循线

第一次到新场地时，按第 1～8 步操作。如果 Office5 已经建好，直接看文末的「下次回到 Office5」。

**操作位置**　在 NoMachine 里打开 Jetson 桌面，按 `Ctrl+Alt+T` 打开终端。下面的命令都可以在同一个终端逐条执行。`dogroute` 会自动加载 ROS 环境。看到以“预期输出”标注的内容时，只需对照屏幕。

**地图位置**　这次的地图叫 `Office5`，保存在 Jetson 的 `/home/nvidia/robot_ws/maps/Office5/`。名称区分大小写，后续都用大写的 `O`。

## 1. 选定新地图 Office5

在 Jetson 终端输入：

```bash
dogroute use Office5
```

预期输出：

```text
已选择 Office5：/home/nvidia/robot_ws/maps/Office5
```

此时程序已记住地图名。保存建图结果时会自动创建目录，无需提前执行 `mkdir`。

## 2. 用遥控器走一圈，建立三维地图

先选一个下次还能准确找到的建图起点，例如地面标线的端点。用遥控器让狗站起来，把**机身中心**放在这个位置，记住狗头朝向。可以拍一张照片，供下次定位时对照。

狗站稳后输入：

```bash
dogroute mapping
```

这条命令会在后台启动 FAST-LIO2 和 PGO。终端重新出现 `nvidia@nvidia-desktop:~$` 时，建图仍在继续。

1. 看建图 RViz，等画面上出现现场雷达点云。
2. 用遥控器带狗走过以后准备画线的区域。如果演示路线是“直走、左转、再走到 B”，这次就把两段直道和中间的转弯都走到。
3. 采集完后，回到建图起点，让狗头恢复到开始时的朝向，保持站稳。

这次遥控走过的轨迹用于采集地图。即使采集时走的是往返路线，第 7 步仍然可以画一条从 A 到 B 的单程路线。

## 3. 保存 Office5

狗回到起点后，在同一终端输入：

```bash
dogroute save
```

等终端显示“已保存 … 个关键帧到 `/home/nvidia/robot_ws/maps/Office5`，建图已结束”。关键帧数量以现场输出为准。可以用下面的命令查看结果：

```bash
ls -lh ~/robot_ws/maps/Office5/
```

| 保存结果 | 后续用途 |
| --- | --- |
| `map.pcd` | 三维点云地图，第 5 步定位时使用。 |
| `poses.txt` | 各关键帧的位姿，第 4 步生成二维栅格时使用。 |
| `patches/` | 各关键帧对应的扫描，与位姿配合生成栅格。 |

## 4. 生成 Nav2 的二维栅格地图

```bash
dogroute grid
```

程序读取 Office5 的关键帧数据，生成两个文件：

```text
/home/nvidia/robot_ws/maps/Office5/nav2/map.yaml
/home/nvidia/robot_ws/maps/Office5/nav2/map.pgm
```

`map.pgm` 是黑白灰的栅格图像，`map.yaml` 记录分辨率、原点等信息。看到终端打印 `Office5/nav2/map.yaml` 后即可继续。定位使用第 3 步的三维点云，画线窗口显示这一步生成的二维图。

## 5. 在 Office5 的建图起点做定位

保持狗站立，机身中心在建图起点，狗头与建图开始时同向。输入：

```bash
dogroute locate
```

这条命令启动定位程序，调用 `/localizer/relocalize`，加载 `Office5/map.pcd`，并以建图起点的位置和朝向作为初始估计。原先手动输入的地图路径和全零初值都已包含在这一步里。

预期输出：

```text
Office5 定位已匹配。请在 RViz 核对固定物体和机身朝向。
```

在定位 RViz 中找一两个现场认得出的固定物体，例如墙角或柱子。看实时扫描的轮廓是否与地图上同一物体重合，再对照狗头朝向。如果一直提示等待匹配，先按第 2 步的照片核对位置和朝向，摆回起点后再运行 `dogroute locate`。

## 6. 打开二维地图，让机器狗准备接收控制

先输入：

```bash
dogroute draw
```

等终端显示“Office5 栅格和控制器已就绪”。如果桌面上有两个 RViz 窗口，**选顶部工具栏带有 `Draw Route` 的那个**，左侧地图图层应显示 `Office5 / Nav2 grid`。

然后在终端输入：

```bash
dogroute stand
```

建图时由遥控器操纵。现在这条命令让 SDK 接管并进入零速 Walking，即使狗已经站着，也要完成这次控制权交接。看到 `walking started` 并确认机身站稳后，开始画线。

## 7. 在 RViz 画出 A 到 B 的路线

1. 在画线 RViz 左侧确认 `Fixed Frame` 为 `map`，点击顶部的 **Draw Route**。
2. 找到代表当前机身中心的**红绿坐标轴交点**。把鼠标放在交点上，**按住左键**，这里就是 A。正常起步要求 A 距当前机身中心不超过 20 厘米。
3. 保持左键按下，沿希望机身中心经过的位置拖动。直行段画直，到了希望转弯的位置再拐，**一直拖到 B 才松手**。
4. 松手后检查屏幕上留下的**紫线**。如果有一段画错，再点一次 **Draw Route**，从 A 重新画。松手保存的新线会替换本图上一条路线。

路线自动保存在：

```text
/home/nvidia/robot_ws/maps/Office5/routes/hand_drawn.json
```

**画完时狗仍在原地。** 看清整条紫线、确认是你想走的路线后，再执行下一步。

## 8. 让机器狗沿紫线走到 B

回到 Jetson 终端输入：

```bash
dogroute walk
```

运行中，终端会显示“正在沿线行走”或“正在对准路线方向”，以及距 A 的沿线里程和剩余距离。到终点时会显示：

```text
Arrived at B. 已到达 B
```

**中途手动停下**时，在运行 `dogroute walk` 的那个终端按 `Ctrl+C`。如果狗仍在原线路段附近，想从当前位置接着走，输入：

```bash
dogroute walk --resume
```

**当天收工**时，等循线结束、机器狗停稳，再逐条执行：

```bash
dogroute release
dogroute stop
```

第一条交还机器狗控制权，第二条关闭画线和定位程序。

## 下次回到 Office5

已有地图和栅格可以直接复用。用遥控器让狗站稳，放回当时的建图起点并对准原朝向，然后按下表操作。

| 顺序 | 命令 | 现场确认 |
| --- | --- | --- |
| 1 | `dogroute use Office5` | 终端显示已选择 `Office5`。 |
| 2 | `dogroute locate` | 等待“定位已匹配”，在定位 RViz 核对固定物体。 |
| 3 | `dogroute draw` | 打开带 **Draw Route** 的窗口，检查已保存的紫线。 |
| 4 | `dogroute stand` | 等待 `walking started`，确认机身站稳。 |
| 5 | `dogroute walk` | 狗在 A 附近且紫线正确时启动；想换路线，先按第 7 步重画。 |
