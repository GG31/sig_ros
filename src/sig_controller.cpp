#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sig_ros/MsgRecv.h>
#include <sig_ros/SetWheel.h>


#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )


class SendController : public Controller
{
   public:
      void onInit(InitEvent &evt);
      double onAction(ActionEvent &evt);
      void onRecvMsg(RecvMsgEvent &evt);
      void setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel);
   public:
      RobotObj *my;
      ros::Publisher onRecvMsg_pub;
      ros::Subscriber setWheel_sub;

};


void SendController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   //my->addForce(0,0,500); 
   my->setWheel(wheel->wheelRadius, wheel->wheelDistance);
}


void SendController::onInit(InitEvent &evt)
{
   int argc = 0;
   char** argv = NULL;
   my = this->getRobotObj(this->myname());
   my->setWheel(10.0, 10.0);
   ros::init(argc, argv, "sig_controller_node");//+std::string(this->myname())
   ros::NodeHandle n;
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &SendController::setWheelCallback, this);
   //ros::spin();
   //ros::Rate loop_rate(10);
}

double SendController::onAction(ActionEvent &evt)
{
   //my->setWheelVelocity(10.0,10.0);
   ros::spinOnce();
   return 0.01;
}

void SendController::onRecvMsg(RecvMsgEvent &evt)
{
   sig_ros::MsgRecv msg;
   msg.sender = evt.getSender();
   msg.content = evt.getMsg();
   onRecvMsg_pub.publish(msg);
}

extern "C"  Controller * createController ()
{
   return new SendController;
}
