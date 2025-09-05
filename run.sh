export DISPLAY=192.168.1.12:0.0
source /opt/ros/humble/setup.bash
source ./install/setup.bash
# ros2 launch geodetic_points vio_gps_integration.launch.py

# ros2 launch geodetic_points test_calibration_globe.launch.py

ros2 launch geodetic_points sigle_calibration_node.launch.py

# docker run -it \
#     --gpus all \``
#     --name "humble_jammy" \
#     --net=host \
#     --privileged \
#     -e DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -e LIBUSB_DEBUG=${LIBUSB_DEBUG:-0} \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#     -v "/home/intellif/zlc_workspace:/home/intellif/zlc_workspace" \
#     -v /mnt/nvme0n1:/mnt/nvme0n1 \
#     -v /dev/bus/usb:/dev/bus/usb \
#     -v /run/udev:/run/udev:ro \
#     -w /home/intellif/zlc_workspace/geodetic-points \
#     ros:humble-perception-jammy \
#     /bin/bash