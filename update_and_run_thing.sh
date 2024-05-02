cd
cd JETSON_AQUIRE_LIDAR_GNSS_IMU/
git pull

echo "UPDATED"

sleep 2

cd app/
python3 imu_ros_node_launch.py