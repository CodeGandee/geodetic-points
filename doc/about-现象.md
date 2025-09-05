# GPS-VIO 标定单节点异常现象与成因分析（about-现象）

本文记录使用 `launch/sigle_calibration_node.launch.py` 启动 `gps_vio_calibration_node` 单节点回放 bag 时观测到的异常日志现象、根因分析与快速修复建议，便于团队复盘与对照修复。

## 背景与复现
- 启动：`geodetic-points/launch/sigle_calibration_node.launch.py`
- 可执行：`gps_vio_calibration_node`（入口自 `setup.py` console_scripts 安装）
- 关键参数：`use_sim_time=true`，rosbag `--clock` 回放，时间同步窗口 60s
- 话题：`/gps/fix -> NavSatFix`、`/odom -> Odometry`
- 日志文件：
  - `log/gps_vio_calibration_*.log`
  - `log/calibration_transform_*.log`

## 现象摘要
- 时间同步日志：
  - 早期 drift ppm 与 offset（b）出现极大值并剧烈跳动，随后 time_diff 约稳定在 -60s 左右。
  - 插值命中率逐步爬升至 96%+，但同步对数（pairs）很久停留在 1/100。
- 标定变换日志：
  - 每 ~2s 输出一次 `earth -> odom` 的 Transform。
  - 平移量处于地球尺度（上百万米），且短时间内剧烈变化，连续多次结果差异很大。

## 根因分析（按影响度排序）
1) 变换方向使用错误（核心）
- 刚体配准后保存的是 `earth -> odom`（R_eo, t_eo）。
- 时间同步误差与离群点剔除需要把 VIO(odom) 变到 earth 对比 GPS(ECEF)，应使用 `odom -> earth`：R_oe = R_eoᵀ，t_oe = -R_oe·t_eo。
- 现实现直接用 `current_transform`（即 R_eo, t_eo）去做 `R_eo·vio + t_eo`，方向相反，导致：
  - 时间模型 a/b 被“拉扯”，drift/offset 乱飘；
  - 插值命中虽高但 pairs 难积累；
  - 配准结果不稳定，Transform 发散（与日志吻合）。

2) SciPy 对齐参数顺序颠倒
- `scipy_rigid_alignment` 使用 `R.align_vectors(Q_centered, P_centered)`，返回满足 `R·Q≈P` 的旋转；
- 后续假设为 `R·P≈Q`，参数应为 `R.align_vectors(P_centered, Q_centered)`，否则得到反向旋转。

3) 时间回归数值不稳与“offset”混淆
- 直接对 ~1.7e9 量级时间戳回归，截距 b（offset 字段）随微小斜率扰动剧烈变化；
- 日志中 ±数亿秒的“offset”实为回归截距，非“当前时间差”；当前时间差应看 `time_diff`（约 -60s）。

4) pairs 累积门槛偏紧/同步失真
- 时间模型失真时 `t_vio_corrected` 多在插值窗口边缘，虽命中但位移 < `min_motion_calib=0.05m` 被跳过；或话题步长过小难过阈。

5) 仿真时间/回放设置
- 已正确 `use_sim_time` + `--clock`；两传感器时间基差异需由时间模型吸收，但前两项未修复时会失效。

## 代码定位（关键片段）
- 方向误用：
  - `optimize_time_offset` 误差函数、`remove_outliers` 残差处，应使用 `odom->earth`（R_oe, t_oe）将 VIO 点变换后再与 GPS 比较。
- SciPy 对齐：
  - `scipy_rigid_alignment` 使用 `R.align_vectors(P_centered, Q_centered)` 与 `R·P≈Q` 假设保持一致。

## 修复建议（最小改动）
- 修正方向：保留 `current_transform` 为 `earth -> odom` 供 TF 发布；凡比较 VIO 与 GPS 处，先构造 `R_oe = R_eoᵀ`、`t_oe = -R_oe·t_eo`，用 `R_oe @ vio + t_oe` 对比 GPS。
- 修正 SciPy 参数顺序：`rot_obj, rssd = R.align_vectors(P_centered, Q_centered)`。
- 提升回归稳定性与日志语义：
  - 回归前对时间戳去原点（减去首样本/均值），优先估计 slope；b 可由滑动窗口 `time_diff` 中位数估计。
  - 日志强调 `time_diff` 代表实时差，避免把回归截距称为“offset”。
- 放宽起步门槛：临时将 `calibration.min_motion_threshold` 调低至 0.02m 或采用累计弧长判据。
- 发布策略：未收敛或 `rms > max_error` 不刷新 `/tf_static`，仅发 `/calibration/transform_earth_odom`；收敛后一次性发布静态 TF（TRANSIENT_LOCAL）。

## 预期效果
- 修正 1/2 后：
  - `time_diff` 收敛稳定、`drift_ppm` 合理；`pairs` 快速增长；
  - 标定 RMS 下降并收敛；`earth -> odom` 变换稳定，不再出现百万米级跳变。

## 验证建议
1) 复播同一 bag，`debug_logging=true`。
2) 观察：
   - `Interpolation hit_rate` 高且稳定，`pairs` 从 1/100 快速增长；
   - `time_diff` 平稳，`drift_ppm` 在合理范围（设备相关）；
   - `/calibration/quality` 下降至阈值附近并收敛；
   - 收敛后 `/tf_static` 仅发布一次，`calibration_transform_*.log` 连续帧变化平滑。

## 相关文件
- 源码：`geodetic_points/gps_vio_calibration_node.py`
- 启动：`launch/sigle_calibration_node.launch.py`
- 日志：`log/gps_vio_calibration_*.log`、`log/calibration_transform_*.log`
