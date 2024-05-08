cd

cd catkin_ws/
source devel/setup.bash
catkin_create_pkg GNSS rospy std_msgs

cd GNSS/

echo "copy the files :  package.xml , GNSS_node.py , /launch/start_GNSS_node.launch , and CMakeLists.txt to the GNSS directory"



echo "Press any key to continue after manually installing the rest of configuration."
echo "[PAUSE]"

read -n 1 -s -r -p "Press any key to continue..."


chmod +x GNSS_node.py 


echo "run the file trought launch"
cd
cd catkin_ws/
echo "roslaunch GNSS start_GNSS_node.launch"



echo "if you have problem with permission : "
sudo usermod -aG dialout usuario
echo "and reboot"



echo "sudo reboot"
echo "to test via termina if the gps is working : "
sudo cat /dev/ttyACM0






