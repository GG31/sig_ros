#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ros_command_node");
   ros::NodeHandle n;
   std::string command = "sigserver.sh -w "+ std::string(argv[1]);// Place of xml file
   system (command.c_str());
   
   return(0);
}
