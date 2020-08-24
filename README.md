# Turtlebot_Jetson_Xavier_NX

This README will take you through the details of how get get a turtlebot running using the Jetson Xavier NX. I have then added the additional steps I had to do to get the turtlebot working with my hardware:

- Jetson Xavier NX
- Roomba 560 (Connected via [USB to FTDI Cable](https://shop.irobot.ca/en_CA/ca/communication-cable-for-create-2/4466502.html?cgid=ca))
- Intel RealSense Depth Camera D415 (with standard USB connection)


## Board Setup

Follow the steps provided by Nvidia: [https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit)

If you don't have a monitor, keyboard, mouse available, you can perform a headless setup according to this link:
[https://desertbot.io/blog/jetson-xavier-nx-headless-wifi-setup](https://desertbot.io/blog/jetson-xavier-nx-headless-wifi-setup)


## Uninstalling OpenCV:
The Jetson Xavier NX comes preinstalled with OpenCV4 so that you can do some interesting opencv examples right off the bat. This causes problems with ROS as (at the time of writing this) ROS runs off of OpenCV3. If you do want to keep the current install of OpenCV then I suggest you try following this [issue](https://github.com/ros-perception/vision_opencv/issues/272) but I'm not sure if this will work completely with the rest of the steps.

```bash
sudo apt-get purge '*opencv*'
```


## Upgrade system
You should probably upgrade the system. You can also use the GUI to do so.

```bash
apt-get update
apt-get upgrade
```

## Installing ROS Melodic
Install ROS based off the ROS installation steps [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Make sure to install the Desktop-Full Install. Additionally you ***do not*** have to run the line echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

## Installing Turtlebot
The installation followed basic turtlebot steps for the most part but there is some changes that I had to make which will be shown by the modification section.
### Basic steps

Below are the list of steps I took to install Turtlebot, it was for the most part based off this [link](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation). I made modifications to use melodic, as well as some changes to ignore errors. **Do not run catkin_make for turtlebot yet.**

```bash
mkdir ~/rocon
cd ~/rocon
wstool init -j6 src https://raw.github.com/robotics-in-concert/rocon/release/indigo/rocon.rosinstall
source /opt/ros/melodic/setup.bash
rosdep install --from-paths src -i -y -r
catkin_make


mkdir ~/turtlebot
cd ~/turtlebot
wstool init src https://raw.github.com/turtlebot/turtlebot/melodic/turtlebot.rosinstall -j6
source ~/rocon/devel/setup.bash
rosdep install --from-paths src -i -y -r
```
Note: I didn't install kobuki as I was running off a roomba base, but if you do want to use kobuki just add the kobuki lines in between the rocon and turtlebot as shown by the normal install [link](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) with the correct modifications.


### Modifications
The modifications will be much easier if you clone this repository if you haven't done so. The opencv folder that is in this repo was copied from another instance of turtlebot I was trying on a VM. For some reason the opencv folder was not created from the previous steps on the Jetson so we will have to copy them over.
```bash
cd ~
git clone https://github.com/johnstonbrendan/Turtlebot_Jetson_Xavier_NX.git
sudo cp -r ~/Turtlebot_Jetson_Xavier_NX/opencv/ /usr/include/
```
Also had to install pyqt5:
```bash
sudo apt-get install pyqt5-dev-tools 
```

Then I had to change some of the turtlebot packages to run from source from their melodic branches.
```bash
cd ~/turtlebot/src
rm -r turtlebot_create_desktop
git clone https://github.com/NU-MSR/turtlebot_create_desktop.git
cd turtlebot_create_desktop
git checkout melodic
cd ~/turtlebot/src
rm -r kobuki_desktop
git clone https://github.com/yujinrobot/kobuki_desktop.git
cd kobuki_desktop
git checkout melodic
```

#### Replacing the driver to work with the roomba
It seemed that the driver built into turtlebot didn't work with my roomba 530. So I decided to replace the driver with another driver I found from this [link](https://github.com/jacobperron/create_robot). This step will highly depend on what base you are using for the robot. I used the 530 which ran of the [600 OI](https://www.irobotweb.com/~/media/MainSite/PDFs/About/STEM/Create/iRobot_Roomba_600_Open_Interface_Spec.pdf).

First I cloned the repo:
```bash
cd ~/turtlebot/src
git clone https://github.com/jacobperron/create_robot.git
```

Then edited the launch file for the mobile_base.launch.xml file so that it used the create_robot driver with the correct parameters. You can edit it manually to fit the changes shown in this repo file or you can just replace the existing file:
```bash
cd ~/turtlebot/src/turtlebot/turtlebot_bringup/launch/includes/roomba/
mv mobile_base.launch.xml backup_mobile_base.launch.xml
cp ~/Turtlebot_Jetson_Xavier_NX/mobile_base.launch.xml mobile_base.launch.xml
```
Summary of what this new file has is that it is essentially using a section of the code found in the [create_2.launch](https://raw.githubusercontent.com/jacobperron/create_robot/melodic/create_bringup/launch/create_2.launch) file in the [create robot git](https://github.com/jacobperron/create_robot)

Now finally we can build:
```bash
cd ~/turtlebot
source ~/rocon/devel/setup.bash
catkin_make
```

After it builds you have some final setup steps to change permissions as well as having the turtlebot running for each new terminal with the correct parameters.
```bash
echo “export TURTLEBOT_3D_SENSOR=realsense_d415” >> ~/.bashrc
echo ”export TURTLEBOT_BASE=roomba” >> ~/.bashrc
echo “export TURTLEBOT_STACKS=circles” >> ~/.bashrc
echo “export TURTLEBOT_SERIAL_PORT=/dev/ttyUSB0” >> ~/.bashrc
echo "source ~/turtlebot/devel/setup.bash" >> ~/.bashrc
sudo usermod -a -G dialout $USER
```
Then reboot the computer for the permission change to come into effect
```bash
sudo reboot
```

### Testing movement
Now you should be able to run the teleop for the turtlebot. Plug in the Jetson to the roomba via the usb to ftdi cable. Run the below commands:
```bash
export TURTLEBOT_3D_SENSOR=kinect
roslaunch turtlebot_bringup minimal.launch
```
Note: you will not want to run the export 3d sensor lines once you have completely setup the camera (see camera section) but for now to just get going we will set the 3d sensor to kinect.

Then in a new terminal:
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```
Now you should be able to move the robot around with the keyboard. Watch out the roomba doesn't drag your Jetson around.

## Camera Setup
### SDK Install
Followed these [steps](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) to get the sdk working.
### Firmware update
Open the real sense viewer:
```bash
realsense-viewer
```
Update the firmware by going to more and then install recommended firmware. If you are running in headless mode (without a monitor) then you are recommended to do this on another laptop. 


### ROS Setup
You can simply install the packages for realsense by running the below.
```bash
sudo apt-get install ros-melodic-realsense2-camera
sudo apt-get install ros-melodic-realsense2-description
```
A lot of documentation for realsense and ros can be found on the git [here](https://github.com/IntelRealSense/realsense-ros)

### Adding the camera to the robot model
In this repo there is a custom robot description that basically adapts the normal kinect version of the file. You can copy over the description by running:
```bash
cd ~/turtlebot/src/turtlebot/turtlebot_description/robots/
cp ~/Turtlebot_Jetson_Xavier_NX/roomba_circles_realsense_d415.urdf.xacro roomba_circles_realsense_d415.urdf.xacro 
```
Note: the roomba_circles_realsense_d415.urdf.xacro file is made based off where the kinect is placed on the turtlebot. You should change the parameters origin to represent where the sensor really is on your robot.

Now if you open a new terminal and run:
```bash
roslaunch turtlebot_bringup minimal.launch
```
and in a new window run:
```bash
roslaunch turtlebot_rviz_launchers view_robot.launch 
```
You should be able to see a model of the robot with the realsense camera on it.

If you want to run the camera at the same time you can run:
```bash
roslaunch realsense2_camera rs_camera.launch 
```
Then going to rviz and checking the depthcloud option. Then going to topic and selecting the /camera/depth/image_rect_raw topic. You should be able to see a depth image of what the camera is seeing projected from the robot model's position of the realsense camera.

Later on we will want this additional step of launching the rs_camera.launch file to be done automatically when it is required, and this will be shown in the next section.

## SLAM and Autonomous Navigation
To get SLAM and autonomous navigation working we'll be using the built in gmapping and amcl demos that come with turtlebot. Once again there will be a few adaptions to be made to get this to work.

### Pointcloud to laserscan
Firstly the gmapping and amcl work off the laser scan topics. So to get a laser scan topic (usually produced by a Lidar) we utilize a package called pointcloud to laser scan. Normally you can also use the depth image to laser scan package, but the pointcloud was easier for me and I had enough computational power to do so.

Install the package with:
```bash
sudo apt-get install ros-melodic-pointcloud-to-laserscan
```

### Camera launch file
WHen gmapping and amcl demo launch files are called the depend on the 3d sensor launch file, which selects a specific launch file according to the TURTLEBOT_3D_SENSOR setting. So we need to create our own d415 camera launch file. This file has been written in this repo and was written based mostly off other launch files as well as the rs_camera launch and the pointcloud to laserscanner file.

Copy this repo launch file for camera into respective folder:
```bash
cd ~/turtlebot/src/turtlebot/turtlebot_bringup/launch/includes/3dsensor/
cp ~/Turtlebot_Jetson_Xavier_NX/realsense_d415.launch.xml realsense_d415.launch.xml 
```
Now when you run:
```bash
roslaunch turtlebot_bringup 3dsensor.launch 
```
and in a new window:
```bash
roslaunch turtlebot_rviz_launchers view_robot.launch 
```
Then checking the scan options you should be able to see the output of the sensor as a laser scan under the /scan topic.

Note: The parameters in the ~/turtlebot/src/turtlebot/turtlebot_bringup/launch/includes/3dsensor/realsense_d415.launch.xml file were not tuned to match the d415 camera's actual specifications. This won't prevent the bot from working but if you want stronger performance I would advise tuning the parameters related to the pointcloud_to_laserscan node seen at the top of the file.

### Gmapping and AMCL
Now we are just going to take the kinect parameter files and copy them for the realsense_d415 for both gmapping and amcl
```bash
cd ~/turtlebot/src/turtlebot_apps/turtlebot_navigation/launch/includes/gmapping/
cp -r -L kinect_gmapping.launch.xml realsense_d415_gmapping.launch.xml 

cd ~/turtlebot/src/turtlebot_apps/turtlebot_navigation/launch/includes/amcl/
cp -r -L kinect_amcl.launch.xml realsense_d415_amcl.launch.xml 
```
Note: Ideally both realsense_d415 launch files (gmapping and amcl) should be customized for the d415 camera specification instead of being just copies of the kinect launch files. But for now it should get it working. 

Additionally we are going to copy the costmap parameter file.
```bash
cd ~/turtlebot/src/turtlebot_apps/turtlebot_navigation/param/
cp -r -L kinect_costmap_params.yaml realsense_d415_costmap_params.yaml
```

### Speed
The roomba runs a little to fast on the defualt parameters (at least for my liking) so the smoother file was adjusted.
```bash
cd ~/turtlebot/src/turtlebot/turtlebot_bringup/param/defaults/
mv smoother.yaml backup_smoother.yaml
cp ~/Turtlebot_Jetson_Xavier_NX/smoother.yaml smoother.yaml
```
### Running
If the bot is all setup mechanically then you should be able to run the gmapping demo according to this [link](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM). The demo assumes that you have a workstation but you can have this all running on the jetson board if you desire (see interfacing section). Then with the same map you should be able to run amcl using the steps in this [link]. (http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map)


## Interfacing
The best way to have this setup would be to have the typical workstation and network setup according to this [link](http://wiki.ros.org/Robots/TurtleBot/Network%20Setup). This requires the Jetson as well as another computer running ROS. An additional method was to have everything running on the jetson and then using a remote desktop. Attempt with nomachine seemed to be a bit problematic but there was a way I found it to get it working.
### Workaround 1
Install nomachine according to this [link](https://www.nomachine.com/download/download&id=111&s=ARM) everytime you boot the jetson, have it plugged into a monitor. Then no machine into the jetson (from another computer). Then unplug the external monitor and it should get working fine. If you don't plug in the monitor before using no machine you will find the display to be extremely small. Workaround 2 will get the display working correctly without having access to tools such as rviz.
### Workaround 2
This is a proper installation of nomachine however it doesn't allow you to use tools such as rviz on the jetson which may be problematic for some people so if you want to use this workaround you probably want to have the workstation and network setup according to this [link](http://wiki.ros.org/Robots/TurtleBot/Network%20Setup).
 
Install no machine according to this [link](https://www.nomachine.com/AR02R01074) Make sure to just comment out the DefaultDesktopCommand line instead of completely replacing it incase you want to revert your changes. Also use gedit instead of vim.
That's it. If you want to reverse what you did uncomment your line in the node.cfg file and recomment the new line you wrote as well as running:
```bash
sudo systemctl set-default graphical.target 
```
Then reboot.

## Moving forward
### Mechanical
Steps moving forward should be (depending on your mechanical design) is to have a stable body for the robot and a way to hold the camera in a better fixed position that is held level. Since the current design is just a cardboard box.

### Electrical
Have the Jetson powered by a portable battery source such as [this](https://www.amazon.com/TalentCell-Rechargeable-11000mAh-14500mAh-26400mAh/dp/B016BJCRUO/ref=mp_s_a_1_5?dchild=1&keywords=talentcell+rechargeable+12v&qid=1594243507&sprefix=talentce&sr=8-5).
Or designing a circuit to have the jetson run off the roomba's battery, which will be interesting as there seems to be a fuse in the power line from the ftdi connector that is giving people trouble.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)