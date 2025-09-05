# VIO 缓冲不足与话题映射问题（GPS 回调持续提示 VIO buffer 太小）

## 现象
- 日志大量出现：`GPS callback: VIO buffer too small (0 < 2), skipping`。
- 与此同时，/calibration 与 /tf_static 日志没有有效数值输出。

## 可能原因
- /odom 未接入（话题名或命名空间不匹配），或 bag 中对应话题不存在；
- bag 开始时 GPS 先到而 VIO 尚未产生，导致早期持续跳过；
- 时间使用 `use_sim_time`+`--clock` 正确，但若 /odom 的时间戳异常或频率过低，也会导致缓冲无法积累。

## 排查建议
- 确认 remap：launch 已将 订阅的`/odom` 映射为默认 `/slamware_ros_sdk_server_node/odom`，需保证 bag 内存在该话题且类型为 `nav_msgs/Odometry`；
- 在回放时检查：`ros2 topic list`、`ros2 topic hz /slamware_ros_sdk_server_node/odom`；
- 先启动节点，待其 ready 再开始 bag 回放；
- 启动阶段可暂时降低 `calibration.min_motion_threshold` 至 0.02 便于快速积累 pairs。
