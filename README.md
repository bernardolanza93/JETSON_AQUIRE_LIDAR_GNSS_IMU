## INSTALLATION PROCEDURE
run this file
```
install.sh
```



# IMU

## Documentation

You can find documentation here :
MTSDK INSTALL DIR/doc/xsensdeviceapi/doc/html/index.html

## Description Environment
setting the environment for sensors
we have installed an external ssd EVO of 1 Tb to save data, it is also linked to download folder, The documents folder inide it is where you want to put the code. 
The libraries will be installed as usually in the parent directory of the operating system
We are using a specific IMU with dev board
to see the specific of IMU MTi 610 go there
```
https://mtidocs.movella.com/development-board
```
A Guide for configuring MTi SDK for Jetson nano:
```
https://base.movella.com/s/article/Interfacing-MTi-devices-with-the-NVIDIA-Jetson-1605870420176?language=en_US
```
Download MTi SDK from here:
```
https://www.movella.com/support/software-documentation
```
## Important:
Run XDA examples in c++ to test the xsens IMU
Regular XDA cannot be compiled using ARM processors. 
Users of ARM platforms (e.g. NVIDIA Jetson) can use the open source XDA examples instead.

## Installing
Download last stable version from here:
```
https://www.movella.com/support/software-documentation"
```
unpack it:
```
tar -xzf MT_Software_Suite...
```
also dependencies that is needed
```
sudo apt update
pip install keyboard
sudo apt install build-essential
sudo apt install libtool
sudo apt install sharutils
sudo apt-get install libusb-1.0-0-dev
```

The MTi USB dongle allows users to connect the robust MTi 600-series (such as the MTi-680G) to a USB port. Support for this accessory is not yet present in older Linux versions of the MT Software Suite. The drivers can be installed separately using:
```
sudo /sbin/modprobe ftdi_sio
echo 2639 0301 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id
```
After installing the drivers, the USB dongle should automatically be mounted to ttyUSB. This can be verified using the dmesg command. If this is not the case any more after rebooting your system, consider adding a udev rule:
Create a file called “95-xsens-ftdi.rules” in the folder /etc/udev/rules.d with the following contents:
```
ACTION=="add" \
ATTRS{idVendor}=="2639" \
ATTRS{idProduct}=="0301" \
RUN{builtin}+="kmod load ftdi_sio" \
RUN+="/bin/sh -c 'echo 2639 0301 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'"
```

then install the .sh file
```
sudo ./mtsdk_linux-xxx-xxxx.x.x.sh
```


link libraries:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/xsens/lib
```

## Getting started

Make sure that you have Xsens MT SDK installed on your system. Before you start using the public XDA, we recommend taking a look at the 'Public XDA Examples', available in the Examples folder from installed MTSDK directory. So you will have some idea how to use it in your application.
All information about how to compile and link a program can be found in either the Visual Studio Solution file or the Makefiles, located in 'src_cpp' example folder. Or you can simply copy xspublic folder, which contains Makefiles, from MTDSK directory to your application directory and start developing.

To compile exapmples: 
Note: If you are using the MTi 10-series or MTi 100-series with a direct USB cable, make sure to have libusb installed, and build the examples using:
```
sudo make HAVE_LIBUSB=1
```
```
sudo make
```
Then you have to move the examples / (or the entire folder) because in the usr folder is not possible generate log files due to the restricted permession of this area of the PC.


# GNSS ARDUSIMPLE RTK

## Preparation of Configuration:
Download the correct configuration in .txt format from the website indicated by Giorgi.
```
https://www.ardusimple.com/configuration-files/
```
The ARDUSIMPLE Bluetooth configuration with firmware version 1.32 is the one of our interest.
Load the configuration after connecting and activating the Ardusimple to the computer via the interface.
## Connection Verification:
Ensure all cables are correctly connected.
Make sure the Ardusimple is connected to the computer.
Before configuring the hotspot, connect the computer to the Wi-Fi network created by the board, which will be named "ESP XBEE".
This connection is necessary to access the IP address 192.168.4.1.
## Access to Web Interface:
Access the specific IP address 192.168.4.1 through the Chrome browser.
Configure your phone's hotspot to provide the web page with the Wi-Fi credentials of the mobile phone (network name and password).
Press the 'Enter' key in the center of the page.
## Board Status and LED Indication:
After connection, the board will be ready to receive data.
The LEDs on the ESP32 socket will display signals:
Flashing blue LEDs indicate activity.
The red LED indicates an internet connection.
The green LED lights up when the GNSS antenna sees the LLeida antenna, improving signal accuracy.
## Board Acquisition via Python:
The board is acquired through a serial connection.
Connect the board to the computer via USB.
Identify the name of the serial port, which is often "dev/tty".
In the Python code, there is a loop that checks which material is transmitting and on which port to listen.
# CUDA
your release: 
```
dpkg -l | grep 'nvidia-l4t-core'
```
want CUDA drivers: 
```
sudo apt-get install nvidia-l4t-cuda
```
or:
```
sudo apt update
sudo apt-get install nvidia-l4t-core
```


# LIDAR 

#

description

## Description
other
```
install.sh
```
•	[thing] : descriptio

-	new : other

## Getting Started

### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

### Installing

* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
```

## Authors

Contributors names and contact info

ex. Bernardo Lanza  
ex. [@DomPizzie](https://twitter.com/dompizzie)

## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)
