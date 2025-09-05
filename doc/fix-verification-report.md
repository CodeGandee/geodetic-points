# GPS-VIO 标定节点修复验证报告

## 修复日期
2025-09-04

## 问题来源
根据 `doc/about-现象.md` 中记录的异常现象和根因分析

## 修复内容

### 1. 变换方向使用错误（核心问题）
**问题**：代码中使用了错误的变换方向，直接使用 `earth -> odom` 变换来转换 VIO 点。

**修复位置**：
- `optimize_time_offset` 函数（第 407-417 行和第 455-459 行）
- `remove_outliers` 函数（第 710-718 行）

**修复内容**：
```python
# 修复前：直接使用 current_transform (earth -> odom)
R = self.current_transform[:3, :3]
t = self.current_transform[:3, 3]
vio_transformed = R @ vio_pos + t

# 修复后：正确计算 odom -> earth 变换
R_eo = self.current_transform[:3, :3]
t_eo = self.current_transform[:3, 3]
# 计算 odom -> earth 变换: R_oe = R_eo^T, t_oe = -R_oe * t_eo
R_oe = R_eo.T
t_oe = -R_oe @ t_eo
vio_transformed = R_oe @ vio_pos + t_oe
```

### 2. SciPy 对齐参数顺序颠倒
**问题**：`scipy_rigid_alignment` 函数中 `align_vectors` 的参数顺序错误。

**修复位置**：`scipy_rigid_alignment` 函数（第 771 行）

**修复内容**：
```python
# 修复前：参数顺序颠倒
rot_obj, rssd = R.align_vectors(P_centered, Q_centered)

# 修复后：正确的参数顺序
# align_vectors(A, B) 找到 R 使得 R @ B ≈ A
# 我们需要 R @ P ≈ Q (VIO -> GPS), 所以参数应该是 (Q, P)
rot_obj, rssd = R.align_vectors(Q_centered, P_centered)
```

### 3. 时间回归数值稳定性
**问题**：直接对大数值（~1.7e9）的时间戳进行回归，导致数值不稳定。

**修复位置**：`update_drift_parameter` 函数（第 545-557 行）

**修复内容**：
```python
# 修复前：直接使用原始时间戳
times_ros = np.array([t_ros for t_ros, _ in self.gps_corrections])
times_corrected = np.array([t_corr for _, t_corr in self.gps_corrections])
slope, intercept, r_value, _, std_err = linregress(times_ros, times_corrected)

# 修复后：对时间戳进行中心化处理
t_mean = np.mean(times_ros)
times_ros_centered = times_ros - t_mean
times_corrected_centered = times_corrected - t_mean
slope, intercept, r_value, _, std_err = linregress(times_ros_centered, times_corrected_centered)
# 调整截距回原坐标系
new_b = intercept + t_mean * (1.0 - slope)
```

## 验证结果

### 构建验证
- 执行 `build.sh` 脚本成功
- 构建时间：1.29s
- 无编译错误

### 运行验证
- 执行 `run.sh` 脚本启动节点
- 节点正常启动，日志显示：
  - GPS-VIO Calibration Node initialized (Fixed Version)
  - 参数正确加载：buffer=60.0s, convergence=0.5m, min_pairs=5
  - 文件日志正常创建

### 测试数据问题
- 测试 bag 文件中缺少 VIO odometry 数据（`/slamware_ros_sdk_server_node/odom` 话题）
- 因此无法完全验证时间同步和标定功能
- 但节点能正常运行，无崩溃或异常

## 预期改进效果

修复后的代码应该能够：
1. **时间同步稳定**：drift_ppm 和 offset 不再出现极大值和剧烈跳动
2. **配对快速累积**：sync pairs 能从 1/100 快速增长
3. **标定收敛**：RMS 误差下降并稳定收敛
4. **变换稳定**：earth -> odom 变换不再出现百万米级跳变

## 建议后续验证

1. 使用包含完整 GPS 和 VIO 数据的 bag 文件进行测试
2. 观察以下关键指标：
   - Interpolation hit_rate 是否达到 90%以上
   - pairs 数量是否持续增长
   - time_diff 是否稳定
   - calibration quality (RMS) 是否收敛到阈值以下
3. 检查日志文件：
   - `./log/gps_vio_calibration_*.log`
   - `./log/calibration_quality_*.json`
   - `./log/calibration_transform_*.log`

## 总结

成功完成了 `gps_vio_calibration_node.py` 中关键问题的修复：
- ✅ 修正了变换方向使用错误
- ✅ 修正了 SciPy 参数顺序
- ✅ 改进了时间回归的数值稳定性
- ✅ 代码成功构建并运行

修复后的代码理论上应能正确执行 GPS-VIO 标定，但需要完整的测试数据进行最终验证。