# 离线 GPS-VIO 标定与可视化计划

目标：将 `geodetic_points/gps_vio_calibration_node.py` 的核心算法从 ROS2 节点剥离为离线 Python 脚本，直接处理 ros2 bag 中的 /slamware_ros_sdk_server_node/odom 和 /cbs_gnss 主题，完成时间同步 + 刚体标定，并对齐后在二维/三维中可视化对齐效果。

## 子任务
- 脚本1：bag 主题抽取器（只读 rosbag2，导出为轻量 CSV/NPZ）
  - 输入：bag 目录路径
  - 输出：`results/` 下两个文件：`odom.csv`（t,x,y,z,qx,qy,qz,qw,frame）、`gps.csv`（t,lat,lon,alt,status,...）
- 脚本2：离线标定 + 可视化
  - 读取脚本1导出的 CSV
  - LLA→ECEF
  - 时间模型拟合（a,b）与窗口内插
  - Kabsch/SVD 刚体配准得到 Earth→Odom
  - 套用变换：将 Odom→Earth 后，与 GPS(ECEF) 对齐
  - 可视化：
    - 2D: XY 平面残差与轨迹
    - 3D: 轨迹与对齐效果（可选）
  - 导出：
    - `results/calib_transform.json`（R,t,a,b,rms）
    - `results/plots/*.png`

## 可能用到的包
- 核心：numpy, scipy, pandas, pyproj, matplotlib
- 读取 rosbag2：
  - 方案：使用 rosbag2_py（需 ROS2 环境；我们优先 A 以脱离 ROS2）

## 建议参考处理ros2bag的脚本
- 脚本：/home/intellif/zlc_workspace/geodetic-points/scripts/bag2kml.py
- 核心：怎么使用mate.xml和db3从ros2bag中读取主题的内容的

## 数据规范
- 时间戳统一为 float 秒（从 ROS stamp 转换）
- 坐标：
  - GPS: WGS84 LLA 
  - VIO/Odom: 直接使用位置 (x,y,z)和方向

## 标定流程（离线）
1) 读入 CSV：t_odom, p_odom；t_gps, lla
2) LLA→ECEF：p_gps_ecef
3) 时间同步：搜索/拟合 a,b 使得 p_odom(t=a*t_gps+b) 与 p_gps_ecef 最近
4) SVD/Kabsch：解 odom→earth 的 R,t
5) 求 earth→odom 作为最终发布/保存
6) 评估 RMS、残差直方图
7) 保存参数与可视化

## 目录与产物
- `scripts/experiment/extract_topics_from_bag.py`
- `scripts/experiment/offline_gps_vio_calibration.py`
- `results/odom.csv, gps.csv, plots/*.png, calib_transform.json`

## 验证
- 用 `/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837` 作为输入
- 主题：`/slamware_ros_sdk_server_node/odom`, `/cbs_gnss`
- 期望：
  - RMS < 若干米（根据数据质量可配置）
  - 可视化中两条轨迹重叠良好

## 后续
- 将离线标定结果回写到 ROS2 节点使用（保存 earth→odom 到文件，再由节点读取）
- 增加 RANSAC/鲁棒配准（可选）
