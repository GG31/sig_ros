sig_ros is a package to interface the SIGverse world with ROS. Indeed, with this package you will be able to use SIGverse through ROS.
user is an example of how to use the sig_ros package. It's the clean up task.

1. Put sig_ros and user in the src folder of your workspace.
2. Change the absolute links on user/xml/CleanUpDemo2014.xml (5), sig_ros/src/ros_controller.cpp (1)
3. Go to user/xml directory
4. Run : rosrun sig_ros ros_controller
5. Start simulation
6. Run : rosrun user RobotCommand
