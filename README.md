
## INSTALLATION PROCEDURE
run this file
```
install.sh
```

## AZURE KINECT
required ros installed
download and install sdk manager for azure kinect following :
https://github.com/valdivj/Azure-for-Kinect-Jetson-nano

download and copy to the catkin_ws / src folder the ros azure drivers from the website guide:
https://scazlab.github.io/azure_setup_guide.html


## ROS 1 Noetic install guide:
Important Note: Due to its upcoming end-of-life, consider ROS 2 (Foxy) for new projects as it offers better performance and support.
ROS Noetic will reach its end-of-life in May 2025

Take into account that you have to flash the jetson without the last jetpack version (6.0..) mounting ubuntu 22. It is preferred also to flaseh with SDK manager installed onto a maximum UBUNTU standalone computer with maximum the 20 LTS versione. For newer version more work is needed. So recapping:
### host machine : Ubuntu 20
### Jetpack versione 5.1.13 (max)
### Ubuntu version on jetson : Ubuntu 20

Follow the official guide to install ROS 1 NOETIC for linux (UBUNTU 20 max supported):
http://wiki.ros.org/noetic/Installation/Ubuntu

Follow the official guie linked in the previous guide to set up the catkin environment
http://wiki.ros.org/catkin/Tutorials/create_a_workspace
Then install IMU xsens drivers and library->

# IMU

## Documentation

### help here:
https://github.com/xsens/xsens_mti_ros_node

https://github.com/nobleo/xsens_mti_driver



You can find documentation inside directory here :
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
Install the MTi USB Serial Driver (try it i get an error)
```
$ git clone https://github.com/xsens/xsens_mt.git
$ cd ~/xsens_mt
$ make
$ sudo modprobe usbserial
$ sudo insmod ./xsens_mt.ko
```

### then install the .sh file
```
sudo ./mtsdk_linux-xxx-xxxx.x.x.sh
```


After installing the drivers, the USB dongle should automatically be mounted to ttyUSB. This can be verified using the dmesg command. 
```
sudo dmesg | grep tty
```
If this is not the case any more after rebooting your system, consider adding a udev rule:
Create a file called “95-xsens-ftdi.rules” in the folder /etc/udev/rules.d with the following contents:

```
sudo touch /etc/udev/rules.d/95-xsens-ftdi.rules
sudo nano /etc/udev/rules.d/95-xsens-ftdi.rules
```
past this and save it ctrl+w (you can use vim or other text editor 
```
ACTION=="add" \
, ATTRS{idVendor}=="2639" \
, ATTRS{idProduct}=="0301" \
, RUN{builtin}+="kmod load ftdi_sio" \
, RUN+="/bin/sh -c 'echo 2639 0301 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'"
```

The device is recognized, but I cannot ever access the device Make sure you are in the correct group (often dialout or uucp) in order to access the device. You can test this with
```
$ ls -l /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
$ groups
dialout audio video usb users plugdev
```
link libraries:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/xsens/lib
```



# ROS NODE for XSENS MTI DRIVER

http://wiki.ros.org/xsens_mti_driver

Source the environment for each terminal you work in. If necessary, add the line to your .bashrc
```
source devel/setup.bash
roslaunch xsens_mti_driver xsens_mti_node.launch
```

i have created a sh script to lounch it directly from home: run
```
./run_xsens.sh
```
modify the properties of the data format of the node here: 
```
/home/usuario/catkin_ws/src/xsens_ros_mti_driver/param
```
### Properties:
imu/data (sensor_msgs/Imu)
quaternion, angular velocity and linear acceleration

imu/acceleration (geometry_msgs/Vector3Stamped)
calibrated linear acceleration

imu/angular_velocity (geometry_msgs/Vector3Stamped)
calibrated angular velocity

imu/mag (geometry_msgs/Vector3Stamped)
calibrated magnetic field

imu/dq (geometry_msgs/QuaternionStamped)
integrated angular velocity from sensor (in quaternion representation)

imu/dv (geometry_msgs/Vector3Stamped)
integrated linear acceleration from sensor

imu/time_ref (sensor_msgs/TimeReference)
timestamp from device

filter/quaternion (geometry_msgs/QuaternionStamped)
quaternion from filter

filter/free_acceleration (geometry_msgs/Vector3Stamped)
linear acceleration from filter

filter/twist (geometry_msgs/TwistStamped)
velocity and angular velocity

filter/positionlla (geometry_msgs/Vector3Stamped) (MTSS2019.3.2 and later)
filtered position output in latitude (x), longitude (y) and altitude (z) as Vector3

filter/velocity (MTSS2019.3.2 and later)
filtered velocity output as Vector3

temperature (sensor_msgs/Temperature)
temperature from device

pressure (sensor_msgs/FluidPressure)
barometric pressure from device

gnss (sensor_msgs/NavSatFix)
raw 4 Hz latitude, longitude, altitude and status data from GNSS receiver

tf (geometry_msgs/TransformStamped)
transformed orientation 


c++ node files saved here:
```
/home/usuario/catkin_ws/src/xsens_ros_mti_driver/src
```

Building:
- Copy xsens_ros_mti_driver folder from your MT SDK directory into your catkin workspace 'src' folder.
Make sure the permissions are set to o+rw on your files and directories.

- Build xspublic from your catkin workspace:
```
$ pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
```

- Build Xsens MTi driver package:
```
$ catkin_make
```
- Source workspace:
```
$ source devel/setup.bash
```
Running:
- Configure your MTi device to output desired data (e.g. for display example - orientation output)

- Launch the Xsens MTi driver from your catkin workspace:

```
$ roslaunch xsens_mti_driver xsens_mti_node.launch

```

## After the device has been detected, you can communicate with it from another process / terminal window.
For example:
```
$ rostopic echo /filter/quaternion
```
This will result in a continuous stream of data output:
---
header: 
seq: 1386351
stamp: 
secs: 1545223809
nsecs: 197252179
frame_id: "imu_link"
quaternion: 
x: 0.00276306713931
y: 0.00036825647112
z: -0.89693570137
w: -0.442152231932
---

- There is also an example that shows a 3D visualization of the device (orientation data should be enabled in the device):
```
$ roslaunch xsens_mti_driver display.launch
```

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
