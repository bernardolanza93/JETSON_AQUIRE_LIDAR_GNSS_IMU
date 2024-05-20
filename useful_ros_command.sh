
source /opt/ros/noetic/setup.bash
roscore

rosbag play JETSON_AQUIRE_LIDAR_GNSS_IMU/data/zed_20240517_150359.bag --topics "/zedxm/zed_node/imu/data"