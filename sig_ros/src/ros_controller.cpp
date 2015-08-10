#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ros_command_node");
   ros::NodeHandle n;
   /*ros::param::set("robot_description", "/home/gg/catkin_ws/src/user/xml/out.urdf");
   ros::param::set("robot_description", "$(find sig_ros)/robot_desc/bobby.srdf");
   ros::param::set("robot_description", "$(find sig_ros)/robot_desc/bobby.srdf");*/
   //TODO Generate urdf file
   //Load robot_000.urdf urdfs yaml
   std::string command = "sigserver.sh -w "+ std::string(argv[1]);// Place of xml file
   system (command.c_str());
   
   return(0);
}
