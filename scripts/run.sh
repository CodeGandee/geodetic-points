# 提取GPS数据
# cd /home/intellif/zlc_workspace/geodetic-points
# python3 /home/intellif/zlc_workspace/geodetic-points/scripts/experiment/extract_gps_data.py \
#     --bag_path /mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837 \
#     --output_dir /home/intellif/zlc_workspace/geodetic-points/results/gps_output \
#     --format both

# echo "GPS数据提取完成"
# # 提取Odometry数据
# python3 /home/intellif/zlc_workspace/geodetic-points/scripts/experiment/extract_odom_data.py \
#     --bag_path /mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837 \
#     --output_dir /home/intellif/zlc_workspace/geodetic-points/results/odom_output \
#     --format both

# echo "Odometry数据提取完成"

echo "开始提取GPS和Odometry数据"

python3 /home/intellif/zlc_workspace/geodetic-points/scripts/experiment/extract_bag_data.py \
    --bag_path /mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837 \
    --output_dir /home/intellif/zlc_workspace/geodetic-points/results/odom_output \
    --gps_topic /cbs_gnss \
    --odom_topic /slamware_ros_sdk_server_node/odom \
    --format both \
    --enable_logging


