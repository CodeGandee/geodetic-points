# 启动异常分析（gps_vio_calibration_node + rosbag 回放）

本文档记录 `launch/sigle_calibration_node.launch.py` 启动后，`gps_vio_calibration_node` 打印大量如下日志的原因分析与修复建议：

> [WARN] ... GPS callback: VIO buffer too small (0 < 2), skipping

## 现象
- 节点启动正常，参数打印如下：buffer=60s, convergence=0.5m, min_pairs=5。
- 随后高频出现 WARN：`GPS callback: VIO buffer too small (0 < 2)`。
- 说明：GPS 消息在到来，但 VIO `/odom` 回调未在本节点中积累到至少 2 条，因此时间同步与配准无法进行。

## 直接原因（最可能）
1) rosbag 回放多路重映射参数用法不当，导致 `/slamware_ros_sdk_server_node/odom` 未成功重映射为 `/odom`：
   - 当前启动文件中 `ExecuteProcess` 写法：
     ```
     ros2 bag play <bag> --rate <r> --clock \
       --remap /cbs_gnss:=/gps/fix \
       /slamware_ros_sdk_server_node/odom:=/odom \
       /slamware_ros_sdk_server_node/points3d:=/vio/points3d
     ```
   - 在 ROS 2 CLI 中，常见的写法是为每一对映射分别提供一个 `--remap`（有些版本/环境下，只解析第一对，后续裸参数不会被识别为 remap 对）。
   - 结果：GPS 主题可能已成功重映射（因其在第一对），而 `/odom` 与 `/points3d` 映射未生效，bag 仍在原始话题名发布，节点却在监听 `/odom`，因此收不到 VIO → VIO 缓冲区一直为 0。

## 其他可能原因（次要但建议排查）
2) QoS 不匹配：
   - 节点订阅 `/odom` 时使用了默认 QoS（深度 50，可靠 Reliable）。若 rosbag 的发布 QoS 为 BestEffort，则可能不匹配。
   - 表现同样为“订阅不匹配，收不到消息”。

3) 话题名/命名空间偏差：
   - 如果实际 bag 中的话题名与假定不一致（例如 `/odom` 的层级或前缀不同），也会导致收不到消息。

4) sim_time 设置：
   - 回放使用 `--clock` 提供模拟时钟，但节点未设置 `use_sim_time:=true` 时，内部 `now()` 使用墙钟。虽然这不会直接导致 VIO buffer 为 0，但会影响时间戳与同步行为，建议保持一致。

## 验证步骤
- 用 `ros2 topic list | grep odom`、`ros2 topic echo -n 1 /odom` 在回放期间确认 `/odom` 是否有消息。
- 用 `ros2 topic info /odom` 查看 QoS，确认是否为 BestEffort；如是，需与订阅端匹配。
- 用 `ros2 bag info <bag>` 或 `ros2 topic list` 检查 bag 内原始话题名，确认 remap 对是否正确对应。

## 修复建议
A. 修正 rosbag 多路 remap 写法（推荐首修）：
```bash
ros2 bag play <bag_dir> \
  --rate <rate> \
  --clock \
  --remap /cbs_gnss:=/gps/fix \
  --remap /slamware_ros_sdk_server_node/odom:=/odom \
  --remap /slamware_ros_sdk_server_node/points3d:=/vio/points3d
```
- 在 `launch/sigle_calibration_node.launch.py` 的 `ExecuteProcess(cmd=[...])` 中，将三对映射分别以独立的 `--remap` 传入。

B. 如仍无 /odom 消息，考虑 QoS 兼容：
- 临时方案：用 `ros2 topic info /slamware_ros_sdk_server_node/odom` 查看回放端 QoS；若为 BestEffort，可将订阅端（代码中 `/odom` 订阅）调整为 `SensorDataQoS` 或 `ReliabilityPolicy.BEST_EFFORT`（需要代码改动）。
- 或使用 `ros2 bag play` 的 QoS 覆盖文件（`--qos-profile-overrides-path`）使回放端以 Reliable 发布。

C. 可选改进（非必须）：
- 在 node 参数中设置 `use_sim_time:=true` 与回放的 `--clock` 对齐，便于时间同步逻辑收敛。
- 在启动脚本中延迟回放 1-2 秒，确保订阅者已就绪再开始回放。

## 关联代码位置
- `geodetic_points/gps_vio_calibration_node.py`：
  - `gps_callback` 开始即检查 `len(self.vio_buffer) < 2`；当 `/odom` 没收到消息时一直触发 WARN。
  - 订阅 `/odom` 使用默认 QoS（`create_subscription(..., 50)`），可能与回放端 QoS 不兼容。

## 结论
- 从日志模式判断，问题根因最可能是 rosbag 多路 remap 参数未全部生效，导致 `/odom` 未正确重映射至本节点所订阅的话题，进而 VIO 缓冲始终为空。
- 按照上文 A 调整 remap 传参后，若仍异常，再按 QoS 与话题名进行排查。
