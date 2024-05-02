cd
cd JETSON_AQUIRE_LIDAR_GNSS_IMU/
git pull

echo "UPDATED"

sleep 2


cd

cd catkin_ws/

source devel/setup.bash
roslaunch xsens_mti_driver /home/usuario/JETSON_AQUIRE_LIDAR_GNSS_IMU/xsens_mti_node.launch