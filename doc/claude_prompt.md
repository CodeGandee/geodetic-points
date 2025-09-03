<background>
geodetic-points/launch/globe_viz.launch.py是个ros2的launch文件，调用不同的节点
rviz的环境是export DISPLAY=192.168.1.12:0.0，bag使用20倍的速度回放
</background>
<need>
1.geodetic-points/geodetic_points/gps_on_globe_node.py中和geodetic-points/geodetic_points/globe_marker_node.py都接受一样的scale参数，根据scale调整展示marker的大小
2.launch文件中增加指定bag包的回放节点，
</need>
<test>
1.使用/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837中的bag包调用launch文件进行回放测试，
2.参数指定展示不同的gps轨迹，不同的gps轨迹分别测试，rviz进行可视化，按照功能名字保存截图到geodetic-points/results中,按照功能写测试报告到doc中，保存多个
3.不同的gps轨迹合并测试，rviz进行可视化，按照功能名字保存截图到geodetic-points/results中,按照功能写测试报告到doc中，保存一个总的
</test>
<target1>
根据上面的目标先写一个实时计划,最终是要通过launch文件启动bag的回放调用绘制节点在rviz中绘制地球和单条或是多条gps轨迹。
</target1>
<target2>
根据上面的目标先写一个测试的目标，其中单个gps轨迹的rviz截图可以看到地球maker和gps的轨迹信息，综合的多个gps轨迹的rviz截图可以看到多条不同的gps轨迹和地球maker，截图要png图片保存到geodetic-points/results中
</target2>
<target3>
按照目标1的计划，测试，测试的结果和目标2的预期测试结果对齐
</target3>

geodetic-points/geodetic_points/globe_marker_node.py中是对地球体的展示，地球体的尺寸使用scale参数进行等比例缩放，在geodetic-points/geodetic_points/gps_on_globe_node.py中是对gps轨迹点的marker展示，其中scale参数和地球体的scale参数保持一致，也对gps的点进行等比例缩放，在geodetic-points/launch/globe_viz.launch.py中启动节点的时候，地球体随gps中轨迹点的播放进行局部放大查看和跟随，先根据上述的要求写个<how-to-目的>的md开发文档到geodetic-points/doc中

gps_on_globe_node.py中的scale是用来控制gps点转换之后到rviz显示的比例尺，不是点的大小的，其次类似/home/intellif/zlc_workspace/geodetic-points/launch/globe_viz.launch.py增加launch文件来绘制多条gps轨迹，根据上述的目的更新geodetic-points/doc/how-to-scale-and-follow-gps-visualization.md文档

目前geodetic-points/geodetic_points/globe_marker_node.py和geodetic-points/geodetic_points/gps_on_globe_node.py中的scale接收之后影响哪些变量先一块计算，之后在后续的应用中不要再使用这个比例因子进行计算，logger的打印尽量集中在一个函数中，该函数专门做日志打印


使用geodetic-points/launch中的不同启动文件进行测试，scale尺寸建议不超过20米，测试结果以rviz的截图的图像的形式保存到geodetic-points/results中，检查保存图像中gps轨迹和地球体是否都可见和清楚



更新geodetic-points/doc/how-to-scale-and-follow-gps-visualization.md文档，功能要求，其中在geodetic-points/launch启动节点的时候，地球体随gps中轨迹点的播放进行跟随和缩放，测试要求，测试结果以rviz的截图的图像的形式保存到geodetic-points/results中，检查保存图像中gps轨迹和地球体是否都可见和清楚作为评判标准


geodetic-points/doc/how-to-scale-and-follow-gps-visualization.md中去掉不同尺度测试，使用scale:=1/300000进行测试单轨迹和多轨迹测试


geodetic-points/geodetic_points/gps_on_globe_node.py中对self.transformer.transform转换前后的点在一个topic主题中发布出来用来debug，如果不能在一个topic中发布出来，只发布转换之后的点的位置信息

现在rviz中有个地球体在展示，同时有个广东省内某个街道的gps轨迹在展示，启动的launch
/home/intellif/zlc_workspace/geodetic-points/doc/how-to-scale-and-follow-gps-visualization.md中主要实现相机根据gps轨迹进行动态跟随的功能，及rviz显示中保存截图到geodetic-points/results中的功能



现有的geodetic-points/launch/globe_viz.launch.py启动ros节点之后，export DISPLAY=192.168.1.12:0.0下rviz的截图如geodetic-points/results/rviz_screenshot_2025_09_01-20_44_31.png，现在存在的问题是，gps的轨迹集中在几个街道内，在地球体上展示这个轨迹的话一个是缩在一个点内另一个是原尺寸展示资源消耗巨大，当前还是需要在地球体中清晰展示gps轨迹，视角集中在gps轨迹所在的区域，能看到gps轨迹增加点的过程，那么从现状到想要的效果，有哪些方法可以做，find online，查找开源的活跃的方案，也可以是原创方案，在最小改动量和最灵活合适本目的上，推荐解决办法，先写个`how-to-目的`的md文档到geodetic-points/doc中


先根据geodetic-points/doc/how-to-dynamic-gps-tracking-visualization.md中的文档说明，实现方案一，并使用/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837这个bag包回放测试方案一，保存rviz截图到geodetic-points/results中，在rviz的截图中能看到地球体和清晰的gps轨迹

在启动geodetic-points/launch/globe_viz.launch.py之后使用/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837这个bag包以30倍的速率回放测试，保存的rviz截图是geodetic-points/results/rviz_screenshot_2025_09_02-14_54_16.png，如截图展示保存的rviz截图中不能看到清晰的gps轨迹，定位和修改上述的问题，并截图验证，find online,think hard

在geodetic-points/launch/globe_viz.launch.py中point_radius_m大小是50000米，通过scale=10000，最终显示在地球上单个gps点是5米，但是实际对应的gps点还是5000米，两个gps点之间最多差10米，总的gps点的轨迹的长度可能也就在2000米左右，所以所有的gps点通过这种方式显示之后在可视化上就是一个点，分辨不出来是一个轨迹，为了解决这个问题，gps点能分辨出来是一个轨迹，要怎么做，先写一个`howe -to`md文档到/home/intellif/zlc_workspace/geodetic-points/doc中，think hard,find online


现在所在的代码终端是通过ssh远程连接的，可通过export DISPLAY=192.168.1.12:0.0的方式转发ROS2 rviz 到本地终端,
现在启动geodetic-points/launch/globe_viz.launch.py，然后写一个最小程序来保存rviz的截图，其中保存图片的形式和截图内容和geodetic-points/results/rviz_screenshot_2025_09_01-20_44_31.png对齐，find online,think hard


在geodetic-points/scripts写个python脚本实现从ros2 bag中抽取所有的gps轨迹到google earth支持的kml格式文件，
该脚本支持指定bag包路径，例如/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837，支持指定保存kml的文件夹，例如geodetic-points/results，
默认按照topic的名字进行保存，从bag包中读取信息参照/home/intellif/zlc_workspace/slamware_ws/src/localization_fusion/test/helpers/test_gps_trajectory_visualization.py中的
read_bag_metadata和read_ros2_bag函数

将/mnt/nvme0n1/resource/rosbags/slef_bag_20250815_170837中的gps轨迹以google earth支持的kml格式抽取到geodetic-points/results中，分别按照topic的名字保存，




当前geodetic-points/launch/globe_viz.launch.py启动节点之后会造成闪烁，rivz截图如geodetic-points/results/rviz_screenshot_2025_09_02-17_45_01.png左侧面板所示




geodetic-points/launch/globe_viz.launch.py这个脚本现有的功能要改变一下：
1.绘制地球体的脚本在固定位置绘制地球体，不改变地球体的位置
2.绘制gps轨迹的节点用来发布gps的轨迹，不发布tf信息
3.新增相机控制节点，控制渲染的相机在gps轨迹当前点的上方100米的位置，100米是launch参数，这个相机显示rviz中当前相机位置看到的画面



/home/intellif/zlc_workspace/geodetic-points/launch/globe_viz.launch.py中的/home/intellif/zlc_workspace/geodetic-points/geodetic_points/camera_control_node.py的功能合并到/home/intellif/zlc_workspace/geodetic-points/geodetic_points/gps_on_globe_node.py中做相机的发布

在geodetic-points/geodetic_points/gps_on_globe_node.py中在函数gps_cb中要对gps中非法的数据做识别、过滤和log打印waring信息，其次相机定义的位置是一定windows内ecef坐标的中心值的上方，其中最小更新的下限在节点中以参数指定，windows大大小也以参数指定，最后calculate_adaptive_marker_sizes函数在两个pulish中计算了两次，alculate_adaptive_marker_sizes作为单独的线程计算以写锁的方式写入结果，计算之后的结果publish方法通过读锁的方式读取，think hard


/home/intellif/zlc_workspace/geodetic-points/launch/globe_viz.launch.py



geodetic-points/geodetic_points/gps_on_globe_node.py中



