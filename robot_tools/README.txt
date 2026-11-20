========================================================
  机器人二次开发工具集
  Jetson Orin Nano · 2026-11-17 整理
========================================================

目录结构
--------------------------------------------------------

  ~/robot_tools/
    README.txt              本文件

    docs/
      MAPPING.txt           建图完整操作清单
      LOCALIZATION.txt      定位（重定位）操作清单

    scripts/
      view_raw_cloud.sh     一键查看雷达原始点云

    rviz/
      rviz_livox_pc2.rviz   RViz 配置（已配好 /livox/lidar_pc2）

    misc/
      frames_*.pdf/.gv      TF 树导出（调试用，可删）
      wifi_brfore_reboot.txt  早期网络调试记录（可删）

常用命令
--------------------------------------------------------

  查看雷达原始点云（一键）：
      ~/robot_tools/scripts/view_raw_cloud.sh

  建图：
      见 docs/MAPPING.txt

  定位：
      见 docs/LOCALIZATION.txt

系统服务（开机自启，不用管）
--------------------------------------------------------

  livox-lidar.service      雷达驱动（CustomMsg 模式）
  livox-watchdog.service   看门狗（雷达掉电重连自动恢复）
  nxserver.service         NoMachine 远程桌面

  查看状态：
      systemctl status livox-lidar
      systemctl status livox-watchdog
      systemctl status nxserver

  看日志：
      journalctl -u livox-lidar -f
      journalctl -t livox-watchdog -f

工作空间（源码，不要动）
--------------------------------------------------------

  ~/ws_livox/       雷达驱动 livox_ros_driver2
                    + livox_pc2_bridge（CustomMsg→PointCloud2 转换）
                    + livox_pc2_bridge 源码在 src/livox_pc2_bridge/

  ~/ws_slam/        FAST-LIO（fastlio2 / pgo / localizer / hba / interface）

  ~/robot_ws/       地图与诊断文档
    maps/           run1、run2 等地图
    diagnostics/    诊断报告

  ~/uniubi_robot_sdk/      宇泛 C++ SDK
  ~/uniubi_robot_sdk_py/   宇泛 Python SDK
  ~/venvs/uniubi/          宇泛 Python 虚拟环境

重要概念
--------------------------------------------------------

  source = 让当前终端认识某个包（只影响这个终端，不启动程序）
  驱动   = 由 systemd 在后台跑，开机即起，与终端无关

  .bashrc 里只 source 了 ROS 2（/opt/ros/humble），
  雷达和 FAST-LIO 工作空间需要手动 source 或由脚本内部 source。

  消息格式：
    /livox/lidar       CustomMsg    ← FAST-LIO 用
    /livox/lidar_pc2   PointCloud2  ← RViz 用（转换节点产出）
    /fastlio2/body_cloud  PointCloud2 ← FAST-LIO 输出（已去畸变）
    /fastlio2/world_cloud PointCloud2 ← 世界系地图
