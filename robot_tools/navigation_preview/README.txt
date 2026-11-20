Office1 global planning preview — 2026-11-17

Requires the existing FAST-LIO2 + Localizer session, localized against office1,
and its existing body -> base_link transform. This launch does not restart them.

Files on Jetson: /home/nvidia/robot_tools/navigation_preview/
Generated grid: /home/nvidia/robot_ws/maps/office1/navigation_preview_v1/
Original PCD maps and localization source were not changed.

Start once, in a separate terminal:
  bash /home/nvidia/robot_tools/navigation_preview/start_preview.sh
Stop that terminal with Ctrl+C. Do not launch a second copy while it is running.

This starts map_server, planner_server, lifecycle manager and a goal-to-plan
adapter only. It starts no controller, SDK connection or cmd_vel publisher.
The /goal_pose subscriber requests ComputePathToPose, not NavigateToPose.

Current RViz: Fixed Frame = map. Add:
  Map:  /navigation_preview/map
  Map:  /global_costmap/costmap (Alpha around 0.4)
  Path: /navigation_preview/path
Set map durability to Transient Local. Set Path durability to Transient Local
if you want to receive the already computed retained path after adding it.
Use the standard 2D Goal Pose tool publishing /goal_pose, click and drag in a
known free region. This generates a new path; it does not move the robot.

Office1 map projection and preview assumptions:
  - office1 initial IMU height approximately 0.45288 m above a level floor,
    derived from the user's approximately 0.45 m lidar mounting-face height.
  - floor_z = -0.45288 in the office1 map, estimated rather than calibrated.
  - obstacle height band = 0.05 to 0.85 m above the assumed floor.
  - per-keyframe height-band angular rays; unknown space remains unknown.
  - Current Jetson planner: centered 0.63 x 0.45 m rectangle and 0.02 m padding.
    Measurements updated 2026-11-19. The validation below used the earlier geometry.
  - static obstacles only. No live obstacle layer, drop-off detector, stair or
    rail traversal, local controller or motion watchdog has been configured.
  - existing localization success flag is latched; keep checking cloud/map
    alignment rather than treating it as continuous localization health.

Validation:
  - Python syntax checked on Jetson.
  - Quaternion transform and all-direction grid-ray endpoint checks passed.
  - Map server, planner server and global costmap active.
  - Preview goal produced 67 poses, about 1.77 m path length.
  - Sampled path cells were known and below inscribed-obstacle cost.
  - No /cmd_vel topic was present during validation.
  - Detailed initial test is saved on Jetson in validation.json.

Next stages after visual review:
  1. Correct map projection and verify actual payload footprint.
  2. Prepare live obstacle layer, base-reference odometry/velocity and local
     controller, with output isolated from the robot while validating.
  3. Integrate the SDK motion bridge, exclusive control arbitration, timeout
     handling and the verified robot locomotion mode.
  4. Add rail approach/traversal task handling; never treat an ordinary drawn
     path over a rail as a validated autonomous crossing.
