echo "setting the environment for sensors"
echo "we have installed an external ssd EVO of 1 Tb to save data, it is also linked to download folder"
echo "to see the specific of IMU MTi 610 go there: "
echo "https://mtidocs.movella.com/development-board"


echo "Operating system info:"
lsb_release -a
echo "device info:"
cat /proc/device-tree/model
echo "memory  info:"
free -h
echo "ARCHITECTURE:"
uname -m


