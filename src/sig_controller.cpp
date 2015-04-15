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
      //シミュレーション開始時に一度だけ呼出される関数onInitの利用を宣言します
      void onInit(InitEvent &evt);
      double onAction(ActionEvent &evt);
      void onRecvMsg(RecvMsgEvent &evt);
      void setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel);
      void callback(const std_msgs::String::ConstPtr& msg);
   public:
      RobotObj *my;

      double velocity;
      ros::Publisher onRecvMsg_pub;
      ros::Subscriber setWheel_sub;
      ros::Subscriber read_sub;

      double Weel_one;
      double Weel_two;
};


void SendController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   std::cout << "hop" << std::endl;
   std::cout << wheel->wheelRadius << std::endl;
   std::cout << wheel->wheelDistance << std::endl;
   //my->addForce(0,0,500); 
   //my->setWheelVelocity(10.0,10.0);
   //sendMsg("robot_000", "go");
   //std::cout<<"go"<<std::endl;
   //LOG_MSG(("haha"));
}

void SendController::callback(const std_msgs::String::ConstPtr& msg)
{
   my->setWheelVelocity(10.0,10.0); 
   std::cout << msg->data.c_str() << std::endl;
}


void SendController::onInit(InitEvent &evt)
{
   Weel_one = 0.0;
   Weel_two = 0.0;

   int argc = 0;
   char** argv = NULL;
   my = this->getRobotObj(this->myname());
   my->setWheel(10.0, 10.0);
   ros::init(argc, argv, "sig_controller_node_"+std::string(this->myname()));
   ros::NodeHandle n;
   velocity = 1;
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &SendController::setWheelCallback, this);
   read_sub = n.subscribe<std_msgs::String>("read", 1, &SendController::callback, this);
   //ros::Rate loop_rate(10);
}

double SendController::onAction(ActionEvent &evt)
{
   //my->setWheelVelocity(10.0,10.0);
   //ros::spinOnce();
   //1秒おきにonActionが呼び出されます
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
