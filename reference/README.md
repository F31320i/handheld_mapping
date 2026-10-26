# handheld_rig 的定位桥（留档副本，不参与编译）

`livo2_odom_bridge.cpp` 是 **`vector_localization` 包里的那个节点的原样副本**
（2026-10-26 拷出），放在这里只是把它留进版本库，避免它只存在于
`vector_robot` 那棵树上。

- 实际编译、实际被 launch 起来的是 `vector_localization` 里那一份：
  `ros2 run vector_localization livo2_odom_bridge`。
- 这个目录没有 `package.xml`，colcon 不会把它当作包，`colcon build` 也不会编译它。
- 两边唯一的关联是：如果哪天从零重建工作空间、又拿不到 `vector_robot`，
  把这份拷回 `vector_localization/src/` 并在它的 `CMakeLists.txt` 里加一行
  `add_executable` 即可（依赖只有 `rclcpp / nav_msgs / geometry_msgs / tf2 /
  tf2_geometry_msgs`）。
- 它是干什么的：把 FAST-LIVO2 输出的 IMU 体系位姿换算成 `base_link` 位姿发到
  `/odometry/lio`，并直接发布 `odom -> base_link` 的 TF（不经过 EKF，避免 EKF
  把 1 mm 的输入放大成 22 mm 的抖动）。详见 `../handheld_mapping/README.md`。
