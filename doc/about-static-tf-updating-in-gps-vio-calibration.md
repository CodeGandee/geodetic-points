# GPS-VIO 校准：静态 TF 被频繁更新的问题

本文记录基于当前仓库日志与源码回溯得到的现象、分析与建议，聚焦“/tf_static 中的 earth→odom 变换被反复更新”的问题。

## 现象与证据
- 变换持续发布且数值随时间变化：
  - 文件 `log/tf_static_20250904_082525.log` 与 `log/calibration_transform_20250904_082525.log` 显示，从 1755249292 起，每约 2s 发布一次 earth→odom 变换；平移分量数量级达 1e6 米（ECEF 量级），四元数也在持续变化。
  - 示例（节选，见原始日志获取完整上下文）：
    - t=1755249292: translation ≈ [2.68e4, 6.32e6, -8.34e5]
    - t=1755249300: translation ≈ [-2.54e6, 5.11e6, -2.85e6]
    - t=1755249440: translation ≈ [2.27e6, -8.32e5, -5.90e6]
- 与“静态 TF”的语义不符：/tf_static 旨在发布不随时间变化的变换；当前日志显示其在不断更新。
- 源码对应逻辑：
  - `geodetic_points/gps_vio_calibration_node.py` 在 `calibration_timer_callback()` 中每 2s 执行一次刚体对齐，随后调用 `publish_calibration_results()`。
  - 当 `calibration.auto_publish_transform=True` 且（已收敛或 `publish_before_convergence=True` 且误差 < max_error）时，会将 TransformStamped 通过 TRANSIENT_LOCAL QoS 发布到 `/tf_static`，导致在未收敛阶段也频繁更新“静态”变换。

## 影响与风险
- 订阅 `/tf_static` 的下游组件可能接收多版互相矛盾的“静态”变换，影响坐标转换稳定性与可重复性。
- 若多个节点缓存并复用不同时间点收到的静态 TF，可能出现定位或可视化跳变。

## 可能根因
1. 设计层面：在“未收敛或正在优化”的阶段，将估计结果发布到 `/tf_static`。
2. 配置层面：参数 `calibration.publish_before_convergence=True` 会放宽发布条件，促成频繁更新。
3. 数据层面：ECEF 量级的平移是合理的（地心地固坐标系），但在未稳定时发布，会使“静态”变换呈现明显时间变化。

## 建议与可操作检查
- 发布策略：
  - 仅在“明确收敛”（RMS < convergence_threshold）后，向 `/tf_static` 发布一次最终变换；在此之前：
    - 将中间结果仅发布到自定义话题（已存在 `/calibration/transform_earth_odom`），供可视化与监控；或
    - 若确需跟随时间变化的变换，请改用 `/tf`（动态 TF）而非 `/tf_static`。
- 参数建议：
  - 将 `calibration.publish_before_convergence` 设为 `false`（避免未收敛阶段写入 `/tf_static`）。
  - 视数据质量与轨迹几何，适当提高 `calibration.min_pairs`、降低 `calibration.max_error`、或收紧 `calibration.convergence_threshold`，以减少误判发布。
- 运行核对：
  - 用 `ros2 topic echo -n 1 /tf_static` 验证最终仅含一条 earth→odom 的静态变换（或随节点重启仅发一次）。
  - 对比 `/calibration/quality` 与 `/sync/*` 指标，确认在稳定阈值内才执行静态发布。
  - 如使用 rosbag 回放，确认 `/odom` 与 `/gps/fix` 均在播放且时间同步（use_sim_time/clock），否则会导致估计抖动或不收敛。

## 备注：关于“VIO buffer too small”告警
- 过往会话中曾提及该告警，但在本次检索的日志集未复现。若再次出现，通常意味着 `/odom` 数据不足（缓冲 < 2 条）导致时间插值失败，应检查：
  - `/odom` 是否在播放/发布、时间戳是否递增且与仿真时间对齐；
  - `time_sync.buffer_seconds` 是否过小导致过快裁剪。

## 结论
- 当前证据表明：节点在未稳定阶段向 `/tf_static` 反复发布估计的 earth→odom 变换，违背“静态 TF 只应在稳定后发布一次”的约定。建议按上文发布策略与参数进行约束，以恢复 TF 语义与下游稳定性。
