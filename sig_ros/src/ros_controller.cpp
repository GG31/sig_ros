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
   //ros::Subscriber sub = n.subscribe("initSigverse", 1000, chatterCallback); //Generate a controller for each agent with ID
   //ros::Publisher speak = n.advertise<std_msgs::String>("speak", 1000);
   //std_msgs::String mesg;
   //mesg.data = "plop";
   //speak.publish(mesg);
   //ROS_INFO("Fuck you 2");
   // Put them in the xml file
   // Compile all controller
   // Launch sigserver
   std::string command = "sigserver.sh -w /home/gg/catkin_ws/src/user/xml/CleanUpDemo2014Robo.xml";// Place of CleaUpDemo2014Robo.xml 
   //+ std::string(argv[1]);// devel/lib/libsig_ros/RobotInMove.xml
   system (command.c_str());
   
   //ros::Rate loop_rate(100);
   //ros::spin();
   return(0);
}
