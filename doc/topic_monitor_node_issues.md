# topic_monitor_node.py 问题记录与改进建议

本文记录在运行 `topic_monitor_node` 时发现的问题、成因分析及可落地的修复建议，便于后续修复与回归验证。

## 复现信息

- 运行命令：`ros2 run geodetic_points topic_monitor_node`
- 关键输出：
  - 已创建 CSV：
    - `./log/topic_monitor_calibration_transform_earth_odom_YYYYMMDD_HHMMSS.csv`
    - `./log/topic_monitor_tf_static_YYYYMMDD_HHMMSS.csv`
    - `./log/topic_monitor_vio_pose_earth_YYYYMMDD_HHMMSS.csv`
  - 异常堆栈（摘要）：
    - `AttributeError: can't set attribute 'subscriptions'`

## 问题清单与修复建议

1) 运行时崩溃：`AttributeError: can't set attribute 'subscriptions'`
- 现象：在 `__init__` 中执行 `self.subscriptions = []` 时报错。
- 根因：`rclpy.node.Node` 已暴露只读属性（property）`subscriptions`，用户代码对其赋值会触发只读属性保护，导致异常。
- 影响：节点在启动阶段即崩溃，无法继续订阅与记录。
- 修复建议：
  - 将本地持有订阅对象的列表改名为自定义属性，例如 `self._subscriptions = []`；
  - 在 `create_subscriptions()` 中把返回的订阅对象 append 到 `self._subscriptions`；
  - 或者完全不持有，依赖 rclpy 内部持有；但为了显式生命周期管理，建议保留 `self._subscriptions`。

2) QoS 配置不合理，可能导致话题不匹配
- 现状：通用 `qos_profile` 使用 `DurabilityPolicy.TRANSIENT_LOCAL`；仅 `tf_static` 使用单独 QoS。
- 问题：大多数常规话题（如 VIO 位姿、标定等）发布端通常为 `VOLATILE` 而非 `TRANSIENT_LOCAL`。订阅端使用 `TRANSIENT_LOCAL` 可能与发布端 QoS 不兼容，导致订阅失败、收不到消息。
- 修复建议：
  - 将通用 `qos_profile` 的 `durability` 改为 `DurabilityPolicy.VOLATILE`；
  - 保持 `tf_static` 使用 `TRANSIENT_LOCAL`（latched 行为）和较小 `depth=1`；
  - 如有需要，为每个话题提供可配置 QoS 参数。

3) 时间戳记录不准确/不一致
- 现状：记录的 `ros_time` 来源于 `self.get_clock().now().nanoseconds`（节点时钟），而不是消息头 `header.stamp`。
- 问题：当 ROS 时间与壁钟存在偏差（仿真/回放/时间重置）时，记录的 ROS 时间可能与消息实际产生时间不一致。
- 修复建议：
  - 对带 `header` 的消息（`TransformStamped`/`PoseStamped`）使用 `msg.header.stamp` 作为 ROS 时间；
  - 可以同时记录：
    - `ros_time_ns`（来自 `header.stamp`）
    - `wall_time_iso`（`datetime.now().isoformat()`）

4) CSV 写入策略效率较低，且锁内执行 IO 影响实时性
- 现状：每次保存时将缓冲区转成 DataFrame，与历史 DataFrame `concat` 后整表重写 CSV；且 `concat`/写文件在互斥锁内执行。
- 问题：
  - 性能：`concat` + 全量覆盖写入在消息多、运行长时会非常慢、占用内存；
  - 实时性：在锁内做 `pandas` 合并与 IO 会阻塞回调线程，导致消息堆积；
  - 稳定性：异常中断时可能丢失尚未落盘的数据。
- 修复建议：
  - 采用增量追加写入（append）策略：
    - 首次写入含表头（`header=True`），后续 `header=False` 直接追加；
    - 或者使用 `csv.DictWriter`/逐行写入，避免依赖 DataFrame 全量状态；
  - 缩小加锁范围：锁内仅“交换/提取缓冲”，IO 在锁外执行；
  - 在 `destroy_node()` 中做最终同步写入，确保退出前落盘；
  - 可选：周期性滚动文件或限制单文件大小。

5) 未使用的参数与导入
- 现状：
  - `monitor_rate_hz` 参数未被使用；
  - 导入了 `Time`、`Header`、`NavSatFix`、`Odometry` 但目前未使用。
- 建议：
  - 若仅用于定期状态输出，可将 `monitor_rate_hz` 作为状态日志定时器频率；否则移除该参数；
  - 清理未使用的导入，避免 lint 警告。

7) 话题名规范化
- 现状：`/tf_static` 过滤 `earth -> odom`；部分系统可能带前导斜杠或命名空间。
- 建议：
  - 比较前先 `lstrip('/')` 进行规范化，或允许配置目标帧 ID；
  - 将目标父子帧通过参数提供，提升通用性。

8) 其他小问题与改进
- DataFrame 的 `defaultdict(pd.DataFrame)` 虽可用，但首次赋值后再 `concat` 逻辑较绕；如改为纯流式写入可以去掉内存 DataFrame；
- 记录文件旁生成一份元数据（JSON），写入会话时间戳、QoS、话题列表、节点参数，有助于后续分析与复现；
- 为每个话题统计丢包/延迟（可选：按 `header.stamp` 与 wall time 差计算）。

## 建议的代码变更要点（摘录）

- 属性命名：
  - `__init__`: 将 `self.subscriptions = []` 改为 `self._subscriptions = []`。
  - `create_subscriptions()`: 把 `sub` 追加到 `self._subscriptions`。

- QoS：
  - 通用话题使用：`QoSProfile(reliability=RELIABLE, durability=VOLATILE, history=KEEP_LAST, depth=10)`；
  - `tf_static` 保持 `TRANSIENT_LOCAL` + `depth=1`。

- 时间戳：
  - `TransformStamped`/`PoseStamped`：
    - 记录 `ros_time_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec`；
    - 记录 `wall_time_iso = datetime.now().isoformat()`；

- 持久化：
  - 新增私有方法 `_flush_topic_buffer_to_csv(topic, rows)`：负责将传入的行列表以 `append` 方式写入 CSV；
  - `save_topic_data()`：锁内取出并清空 `self.data_buffers[topic]`，锁外调用 `_flush_topic_buffer_to_csv()`；
  - 删除全量 `concat` 与 `self.dataframes[...]` 的强依赖，避免整表重写；

- 改进：
  - 未使用的导入`monitor_rate_hz` 保留，增加一个根据该频率打印统计信息的定时器；否则移除该参数声明与使用。

## 参考验证步骤

1. 应用上述修改后，重新构建并运行：
  - 确认节点启动无 `AttributeError`；
  - 在预期话题上能正常收到消息（可配合 `ros2 topic info -v` 查看 QoS 是否匹配）；
  - CSV 文件应按追加方式增长，退出时数据完整；
  - `tf_static` 能在启动后立即获取到 `earth->odom` 的静态变换条目。

2. 回归指标：
  - 启动时间不再异常中止；
  - 长时间运行（>30min）CPU/内存占用稳定，无持续上涨；
  - 文件大小与写入速率符合预期，无明显阻塞回调现象。

---

备注：尝试查阅 rclpy 官方在线文档以确认 `Node.subscriptions` 为只读属性时遇到页面 404，但从 rclpy 常见用法与错误信息可明确判断该属性为只读且不应在用户代码中直接赋值。实际修复以改名自定义属性即可根治该问题。