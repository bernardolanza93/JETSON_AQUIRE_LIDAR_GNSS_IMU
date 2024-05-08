#!/bin/bash

# Check if the system is Linux
if [ "$(uname -s)" != "Linux" ]; then
    echo "The operating system is not Linux."
    exit 1
fi

# Check Ubuntu version
if [ "$(lsb_release -si)" == "Ubuntu" ]; then
    VERSION=$(lsb_release -rs)
    if [[ $VERSION == "20."* ]]; then
        echo "[OK] The operating system is Ubuntu 20.x."
    else
        echo "The operating system is Ubuntu, but it's not version 20.x. - YOU CAN HAVE SERIOUS PROBLEM WITH ROS"
        exit 1
    fi
else
    echo "The operating system is not Ubuntu."
    exit 1
fi




echo "[AUTO-INSTALL-DEPENDENCIES]"
sudo apt update
pip install keyboard
sudo apt install build-essential
sudo apt install libtool
sudo apt install sharutils
sudo apt-get install libusb-1.0-0-dev
echo "The MTi USB dongle allows users to connect the robust MTi 600-series (such as the MTi-680G) to a USB port. Support for this accessory is not yet present in older Linux versions of the MT Software Suite. The drivers can be installed separately using:"
sudo /sbin/modprobe ftdi_sio
echo 2639 0301 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id
echo "[ATTENTION - UNSTABLE - NOT NECESSARY - START SECTION]"
git clone https://github.com/xsens/xsens_mt.git
cd ~/xsens_mt
make
sudo modprobe usbserial
sudo insmod ./xsens_mt.ko
echo "[ATTENTION - UNSTABLE - NOT NECESSARY - END SECTION]"
echo "----------------------------------------------------"
echo "[IMPORTANT - HARDCODED MODE]"
echo "[DOWNLOAD DIY]"
echo "download the last ARM-64 drivers of Mti xsens IMU from the site above:"
echo "MTi Products-Latest stable software fro Linux ARM-64"
echo "https://www.movella.com/support/software-documentation"
echo "unpack the version you have downloaded with tar:"
echo "tar -xzf MT_Software_Suite..."
echo "enter the download directories and run the sh installartion file: <MTSDK>"
echo "cd /Downloads"
echo "sudo ./mtsdk_linux-xxx-xxxx.x.x.sh"

# Display a pause message
echo "Press any key to continue after manually installing the MTI driver and libs."
echo "[PAUSE]"
# Pause the script until the user presses a key
read -n 1 -s -r -p "Press any key to continue..."
echo "[CONTINUE]"
# Here you can insert the code for manually installing the libraries, if needed
# Continue with the rest of the script
echo "Continuation of the script after manual installation of libraries."




echo "After installing the drivers, the USB dongle should automatically be mounted to ttyUSB. This can be verified using the dmesg command"
sudo dmesg | grep tty


echo "If this is not the case any more after rebooting your system, consider adding a udev rule: Create a file called 95-xsens-ftdi.rules in the folder /etc/udev/rules.d with the following contents:"


echo "sudo touch /etc/udev/rules.d/95-xsens-ftdi.rules"
echo "sudo nano /etc/udev/rules.d/95-xsens-ftdi.rules"
echo "past this and save it ctrl+w (you can use vim or other text editor"
file="resources/95-xsens-ftdi.rules"

# Verifica se il file esiste
if [ -f "$file" ]; then
    cat "$file"  # Stampare il contenuto del file
else
    echo "Il file $file non esiste."
    exit 1
fi
echo "a copy of this file is here:"
echo "resources/95-xsens-ftdi.rules"



# Display a pause message
echo "Press any key to continue after manually installing the MTI driver and libs."
echo "[PAUSE]"
# Pause the script until the user presses a key
read -n 1 -s -r -p "Press any key to continue..."
echo "[CONTINUE]"
# Here you can insert the code for manually installing the libraries, if needed
# Continue with the rest of the script
echo "Continuation of the script after manual installation of libraries."

echo "The device is recognized, but I cannot ever access the device Make sure you are in the correct group (often dialout or uucp) in order to access the device. You can test this with"
ls -l /dev/ttyUSB0
echo "should be lie that:"
echo "crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0"
groups
echo "should be lie that:"
echo "dialout audio video usb users plugdev"
echo "link libraries:"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/xsens/lib
