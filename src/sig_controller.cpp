#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sig_ros/MsgRecv.h>
//Robot Msg
#include <sig_ros/SetWheel.h>
#include <sig_ros/SetWheelVelocity.h>
#include <sig_ros/SetJointVelocity.h>
//Srv
#include "sig_ros/getTime.h"

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )


class RobotController : public Controller
{
   public:
      void onInit(InitEvent &evt);
      double onAction(ActionEvent &evt);
      void onRecvMsg(RecvMsgEvent &evt);
      //Robot
      void setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel);
      void setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel);
      void setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg);
      //Srv
      bool getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res);
   public:
      RobotObj *my;
      
      ros::Publisher onRecvMsg_pub;
      
      //Robot
      ros::Subscriber setWheel_sub;
      ros::Subscriber setWheelVelocity_sub;
      ros::Subscriber setJointVelocity_sub;
      
      //
      ros::ServiceServer service;
};  
  
void RobotController::onInit(InitEvent &evt)
{
   int argc = 0;
   char** argv = NULL;
   my = this->getRobotObj(this->myname());
   my->setWheel(10.0, 10.0);
   ros::init(argc, argv, std::string(this->myname()) + "_sig_controller_node");//+std::string(this->myname())
   ros::NodeHandle n;
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &RobotController::setWheelCallback, this);
   setWheelVelocity_sub = n.subscribe<sig_ros::SetWheelVelocity>(std::string(this->myname()) + "_setWheelVelocity", 1, &RobotController::setWheelVelocityCallback, this);
   setJointVelocity_sub = n.subscribe<sig_ros::SetJointVelocity>(std::string(this->myname()) + "_setJointVelocity", 1, &RobotController::setJointVelocityCallback, this);
   
   service = n.advertiseService(std::string(this->myname()) + "_get_time", &RobotController::getTime, this);
   //ros::spin();
   //ros::Rate loop_rate(10);
}

double RobotController::onAction(ActionEvent &evt)
{
   //my->setWheelVelocity(10.0,10.0);
   ros::spinOnce();
   return 0.01;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
   sig_ros::MsgRecv msg;
   msg.sender = evt.getSender();
   msg.content = evt.getMsg();
   onRecvMsg_pub.publish(msg);
}

void RobotController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   std::cout << "setWheelCallback" << std::endl;
   my->setWheel(wheel->wheelRadius, wheel->wheelDistance);
}

void RobotController::setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel)
{
   std::cout << "setWheelVelocityCallback" << std::endl;
   my->setWheelVelocity(wheel->leftWheel, wheel->rightWheel);
}

void RobotController::setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg)
{
   std::cout << msg->jointName << " " << msg->angularVelocity << " " << msg->max << std::endl;
   my->addForce(0.0,0.0,500.0);
   //my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
}
//Srv
bool RobotController::getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res)
{
   std::cout << "on getTime" << std::endl;
   res.time = getSimulationTime();
   return true;
}
//End Srv
extern "C"  Controller * createController ()
{
   return new RobotController;
}
