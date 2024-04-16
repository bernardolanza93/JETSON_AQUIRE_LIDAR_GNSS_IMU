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


echo "Download last stable version from here:"
echo "https://www.movella.com/support/software-documentation"

echo "unpack it:"
echo "tar -xzf MT_Software_Suite..."

echo "then install the .sh file"
echo "sudo ./mtsdk_linux-xxx-xxxx.x.x.sh"

echo "if you have this error install..."
echo "uudecode could not be found' sharutils"
echo "sudo apt update"
echo "sudo apt install sharutils"

echo "make xsens syteme wide with this command:"
echo "sudo ldconfig /usr/local/xsens"

echo "link libraries:"
ech "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/xsens/lib"


pip install keyboard



