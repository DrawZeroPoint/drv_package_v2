# drv_package_v2.0

![overview](https://github.com/DrawZeroPoint/drv_package_v2/blob/master/supplements/figures/1.png )

Deep Robot Vision 2.0 system for NEU Vision Gear, see the [wiki](https://github.com/DrawZeroPoint/drv_package_v2/wiki) for details.
For the former version of DRV, refer https://github.com/NEU-TEAM/drv_package. 
*We refer this package, aka drv_package_v2 as DRV in the following instructions.*

## 1. Software prerequisites
### 1.1 ROS
We run this package in **indigo**, other versions of ROS *may not* be supported.

### 1.2 py-faster-rcnn
This package is not included in the `drv_package`, you need to get it from <https://github.com/rbgirshick/py-faster-rcnn>, and install it following official guide (the customized Caffe installation part is basicly the same with installing ordinary Caffe when using cmake). Notice that if you use cuDNN 5.0 or above, `git clone https://github.com/donghongwen/py-faster-rcnn_cudnnv5.git` instead and cmake (or make). 
After that, you need download the *VGG16_faster_rcnn_final.caffemodel*  into `DRV/supplements/object_recognize` which you can obtain by executing `./data/scripts/fetch_faster_rcnn_models.sh` in `$PY_FASTER_RCNN`. Notice that the download address of the model has been changed to https://dl.dropboxusercontent.com/s/6joa55k64xo2h68/Makefile.config?dl=0

### 1.3 rosserial
Please refer [Arduino IDE Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) for using rosserial. To install it, we basicly need to 1) `sudo apt-get install ros-indigo-rosserial-arduino` and `sudo apt-get install ros-indigo-rosserial`. 2) Install the library ros_lib. 3) Install the libraries you might not have: FastLed, VerSpeedServo and JY901. The first two can be found in Arduino offical library, the last is provided in DRV/supplements/arduino_control.
We use **rosserial_arduino** to communicate between one Arduino Mega 2560 board and an JY61 6-axis accelerator. The Arduino controls 2 servos, one for pitching the RGBD-camera (Orbbec ASTRA S) and the other for rotating it. The accelerator measures the pitch and yaw angles of the camera. The .ino file loaded on the Arduino board is provided in folder `$DRV/supplements/arduino_control`, you can load it to your board with Arduino IDE <http://arduino.cc/en/Main/Software>.

### 1.4 Orbbec Astra
First clone the repo https://github.com/orbbec/ros_astra_camera.git and https://github.com/orbbec/ros_astra_launch.git into your catkin_ws, then do `catkin_make --pkg astra_camera -DFILTER=ON` in /catkin_ws.
If you get error: `Getting "fatal error: libudev.h: No such file or directory #include <libudev.h>"` when compiling the package, do `sudo apt-get install libudev-dev`. `sudo apt-get install ros-indigo-rgbd-launch` if necessary.
After that, `roscd astra_camera && ./scripts/create_udev_rules` to make astra udev rules. Run it by `roslaunch astra_launch astra.launch`
You can find the calibration files for our devices (Astra S) in folder *DRV/supplements/camera_info* for reference purpose. When running in ROS, this folder should be put into `/home/YOUR_USERNAME/.ros`.

### 1.5 GOTURN (Deprecated)
We used to using GOTURN as our object tracking backbone, however, it performs slow in CPU-only computer such that we trun to Correlation Filter method (KCF). The algorithm using GOTURN is still provided for reference purpose. GOTURN itself is not necessarily needed to run tracking, we only need the trained model *tracker.caffemodel* and prototext *tracker.prototxt* in `DRV/supplements/object_track/`. You can get the model from <https://github.com/davheld/GOTURN>.

### 1.6 NEU_FACE
We provide the face recognition function based on [VGG_Face](http://www.robots.ox.ac.uk/~vgg/software/vgg_face/). To perform face recognition, the caffe model (ex. *finetune_neu_face.caffemodel*) and prototext (ex. *neu_face_deploy.prototxt*) should in `DRV/supplements/face_recognize/`.

### 1.7 gpd
We optionally use [gpd](https://github.com/atenpas/gpd) to generate grasp pose. You may need to install it first to compile DRV. We provide the launch file `drv_gpd_port.launch` to run gpd in `DRV/drv_brain/launch`

## 2 Hardware
To run searching and tracking modules smoothly, a workstation with NVIDIA GPU which has at least 4GB of GRAM is necessary. If the machine has no GPU, it can only run in **simple** mode which means no searching or face recognition can be performed.
The host computer's CPUs run at frequencies exceeding 2.4 GHz. Low frequencies will lead to a delay in point cloud capturing and communication between the workstation and the host PC.
This program has also been tested on multi-machine configurations.

## 3. Installation
1. Clone this repository into catkin_ws/src (we assume your ROS work space has the name *catkin_ws*):
`git clone https://github.com/DrawZeroPoint/drv_package_v2.git`
2. Add the line `source /home/USER/catkin_ws/devel/setup.bash` to your ~/.bashrc.
2. Run `catkin_make --pkg drv_msgs` in ~/catkin_ws to generate header file used by other nodes in DRV.
3. Run `catkin_make` in ~/catkin_ws to make all packages and nodes.
4. Add these two lines into your ~/.bashrc:
 ```
export PYTHONPATH="/home/USER/py-faster-rcnn/lib:/home/USER/py-faster-rcnn/caffe-fast-rcnn/python:${PYTHONPATH}"
export DRV=/home/USER/catkin_ws/src/drv_package_v2
 ```
 **Replace 'USER' with your actual username**

## 4. Usage
1. Run `roscore` first. Plug in the NVG (NEU Vision Gear).
2. Run `roslaunch drv_brain drv_v2.launch` or `roslaunch drv_brain drv_v2_simple.launch` to launch the whole robot vision system on a single machine. The main difference of the two launch files is that the *simple* version doesn't perform searching. Besides, if you let arg "simple_id" in drv_v2_workstation_simple.launch.xml to be true, it will publish object pose for only once for one detection. The whole vision system runs in namespace /vision, and you can change it by modifing the arg "ns" in drv_v2.launch. Meanwhile, you need to pay attention to the launch file drv_v2_host.launch.xml, in which we assume your Arduino board link to USB port "/dev/Arduino_vision", you can make this symlink by following [this instruction](http://www.joakimlinde.se/microcontrollers/arduino/avr/udev.php). Briefly, you need create a udev rule file (a example can be found in folder `DRV/supplements/arduino_control`, notice that your serial is different form ours) for Arduino and put it into the folder `/etc/udev/rules.d`. To make the rules work, you may re-plugin the Arduino. 
3. If you use DRV on multiple machines, run `roslaunch drv_brain drv_v2_host.launch` on host machine (which is carried by the robot), and `roslaunch drv_brain drv_v2_workstation.launch` or `roslaunch drv_brain drv_v2_workstation_simple.launch` on the workstation, to control the robot and process data remotely.
4. Target can be assigned by setting the rosparam: `rosparam set /comm/control/target/label bottle`, `rosparam set /comm/control/target/is_set true`, the order of these settings should not be changed. Here the 'bottle' refers to the target label and can be changed to 'chair', 'person', etc. as long as it belongs to Pascal_VOC categories. FYI, the target setting function as well as more useful functions can be easily realized with [JARVIS](https://github.com/NEU-TEAM/JARVIS), which is an Android app for controlling the NEU household robot. Till now it only supports Android 6.0+.
5. With the target set, the system will automatically run in *search mode* and find the target in the scene. When some objects which likely to be the target were found, the system will request user select to judge the candidates and decide whether continuing searching the target or tracking the selected target. If the target is confirmed, the system will run in *tracking mode* in which the location of the target in 3D space will be generated for manipulating the target.
6. By setting the target *is_set* param to be *false*, the system will run in *wander mode* and do nothing.

## 5. Trouble Shooting
1. If custom message issue occurred when running catkin_make, run `catkin_make --pkg drv_msgs --force-cmake` first to make the msg header files needed, and then run `catkin_make`.
 
## 6. Development details
1. To demonstrate the status of the robot, we use full-color LEDs to show the status with these light code (std_msgs::UInt16) defination:
0: normal (green)
1: warning (yellow) 2: error (red) 3: fatal error (red flash)
10: fully bright (white) 11: extinguished (black) 12: breathing light (flashing purple)

## Author
[Zhipeng Dong](https://github.com/DrawZeroPoint) zhipengdongneu@gmail.com (Discussion about this program is welcome) 
