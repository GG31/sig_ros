#define CONTROLLER
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <SimObj.h>
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
#include "sig_ros/getObjPosition.h"
#include "sig_ros/getPartsPosition.h"
#include "sig_ros/getRotation.h"
#include "sig_ros/getAngleRotation.h"

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )


class RobotController : public Controller
{
   public:
      void onInit(InitEvent &evt);
      double onAction(ActionEvent &evt);
      void onRecvMsg(RecvMsgEvent &evt);
      void onCollision(CollisionEvent &evt); 
      //Robot
      void setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel);
      void setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel);
      void setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg);
      //Srv
      bool getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res);
      bool getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res);
      bool getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res);
      bool getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res);
      bool getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res);
      
   public:
      RobotObj *my;
      
      ros::Publisher onRecvMsg_pub;
      
      //Topic
      ros::Subscriber setWheel_sub;
      ros::Subscriber setWheelVelocity_sub;
      ros::Subscriber setJointVelocity_sub;
      
      //Srv
      ros::ServiceServer service;
      ros::ServiceServer serviceGetObjPosition;
      ros::ServiceServer serviceGetPartsPosition;
};  
