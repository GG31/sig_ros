sig_ros is a package to interface the SIGverse world with ROS. Indeed, with this package you will be able to use SIGverse through ROS.
user is an example of how to use the sig_ros package. It's the clean up task.

* Create a catkin workspace : 
```
mkdir -p ~/catkin_ws/src
```
* Initialize the workspace :
```
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
```
* Clone the git repository
```
git clone https://github.com/GG31/sig_ros.git
```
* Go to the tag vDemoCleanUp
```
git checkout tags/vDemoCleanUp
```
* Change the name of sig_ros folder you've just cloned by src, so you have the tree : 
```
|-- catkin_ws
    |-- src
        |-- sig_ros
        |-- user
    |-- devel
    |-- build
```
* Change the absolute links on catkin_ws/src/user/xml/CleanUpDemo2014.xml there is 5, on catkin_ws/src/sig_ros/src/ros_controller.cpp there is one and on catkin_ws/src/sig_ros/CMakeLists.txt
* Create libsig_ros : 
```
mkdir ~/catkin_ws/devel/lib/libsig_ros
```
* Compile : 
```
cd ~/catkin_ws
catkin_make --pkg sig_ros
catkin_make --pkg user
source devel/setup.bash
```
* Go to ~/catkin_ws/src/user/xml directory
* Run sig_ros : 
```
rosrun sig_ros ros_controller
```
* Connect the SIGViewer and start the simulation
* Run user package : 
```
rosrun user RobotCommand
```
The Robot will move to clean up the house.
