# ws_pct 本地定制（gitee 三方，整包不入库）

`ws_pct` = https://gitee.com/fdsf3e2342/3d_navi_ros2.git （3D 全局规划器，实验性）

恢复步骤：

```bash
cd ~ && git clone https://gitee.com/fdsf3e2342/3d_navi_ros2.git ws_pct
cd ~/ws_pct
git apply ~/third_party/ws_pct_cyvet/tracked-changes.patch   # 已跟踪文件的本地改动
cp ~/third_party/ws_pct_cyvet/cyvet.yaml       src/global_planner/config/
cp ~/third_party/ws_pct_cyvet/cyvet.launch.py  src/global_planner/launch/
```

`status.txt` 记录了当时的未跟踪文件清单，`tracked-changes.patch` 是 5 个已跟踪文件的改动。
