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
pip install keyboard
```

then install the .sh file
```
sudo ./mtsdk_linux-xxx-xxxx.x.x.sh
```

if you have this error install...
```
udecode could not be found' sharutils
```

```
sudo apt update
```
```
sudo apt install sharutils
```


link libraries:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/xsens/lib
```

## Getting started

Make sure that you have Xsens MT SDK installed on your system. Before you start using the public XDA, we recommend taking a look at the 'Public XDA Examples', available in the Examples folder from installed MTSDK directory. So you will have some idea how to use it in your application.
All information about how to compile and link a program can be found in either the Visual Studio Solution file or the Makefiles, located in 'src_cpp' example folder. Or you can simply copy xspublic folder, which contains Makefiles, from MTDSK directory to your application directory and start developing.

To compile exapmples: 
```
sudo make
```
Then you have to move the examples / (or the entire folder) because in the usr folder is not possible generate log files due to the restricted permession of this area of the PC.
# LIDAR 

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
