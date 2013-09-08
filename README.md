###Setup       
* If you haven't already, please install ROS [furte](http://wiki.ros.org/fuerte/Installation/Ubuntu) or [groovy](http://wiki.ros.org/groovy/Installation/Ubuntu) on your Ubuntu. There are other Linux [distributions](http://wiki.ros.org/ROS/Installation) supported by ROS but Leapmotion works best on Ubuntu so far.
* If you are on Ubuntu 12.04 or 12.10, most likely your kernel is outdated for the Leapmotion driver to function. 12.04 LTS now seems to have kernel 3.8 by default but please check by running `uname -a`. If the kernel version is lower than 3.8, please update the kernel image to 3.8.       
             
          sudo apt-get install linux-image-generic-lts-raring      
          sudo apt-get install linux-headers-generic-lts-raring    
          
_Warning: updating the kernel might cause some malfunctionality of the system._       
* Enter `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/where/is/ROSLeap` to `~/.bashrc`, and run `source ~/.bashrc`.
* If you haven't installed the Leapmotion driver for Linux, please run `./setup.sh` in this repository.   
