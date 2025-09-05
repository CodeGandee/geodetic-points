# 离线 GPS-VIO 标定测试计划

## 测试目标
- 验证离线脚本可正确读取 ros2 bag（或其导出的 CSV）
- 验证时间同步与刚体配准的数值稳定性与收敛性
- 验证对齐后在可视化中能清晰看到两条轨迹重合

## 测试输入
- bag 目录：`/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837`
- 主题：
  - `/slamware_ros_sdk_server_node/odom` (nav_msgs/Odometry)
  - `/cbs_gnss` (sensor_msgs/NavSatFix)

## 用例
1) 抽取用例
   - 运行 `extract_topics_from_bag.py` 指定 bag 路径与输出目录
   - 断言：生成 `results/odom.csv` 与 `results/gps.csv`，行数>0，列齐全
2) 标定用例（默认参数）
   - 运行 `offline_gps_vio_calibration.py` 指定 CSV 路径
   - 断言：输出 `calib_transform.json` 含 R(3x3)、t(3)、a、b、rms
   - 断言：`plots/*.png` 文件生成（至少2D轨迹与残差图）
3) 鲁棒性用例（截断/降采样）
   - 仅取 bag 的部分时段，或对 GPS 进行降采样
   - 断言：算法仍能输出合理的变换与 RMS

## 验证标准
- 轨迹重叠直观良好，残差图呈集中分布
- RMS 在合理阈值（例如 < 10 m，可按数据质量调整）
- 输出的 earth→odom 变换可被 ROS2 节点读取使用

## 产出
- `results/calib_transform.json`
- `results/plots/*.png`
- 运行日志（标准输出即可，可选保存到 `results/logs/*.txt`）
