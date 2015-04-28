#ifndef SIM_OBJ_CONTROLLER_H
#define SIM_OBJ_CONTROLLER_H
#define CONTROLLER
#define dDOUBLE
#define USE_ODE
//SIGverse
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <SimObj.h>
#include <Rotation.h>

#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

//Robot Msg
#include <sig_ros/MsgRecv.h>
#include <sig_ros/SetJointVelocity.h>
#include <sig_ros/ReleaseObj.h>
//Srv
#include "sig_ros/getTime.h"
#include "sig_ros/getObjPosition.h"
#include "sig_ros/getPartsPosition.h"
#include "sig_ros/getRotation.h"
#include "sig_ros/getAngleRotation.h"
#include <sig_ros/getJointAngle.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )


class SimObjController : public Controller
{
   public:
      virtual void onInit(InitEvent &evt) = 0;
      virtual double onAction(ActionEvent &evt) = 0;
      virtual void onRecvMsg(RecvMsgEvent &evt) = 0;
      virtual void onCollision(CollisionEvent &evt) = 0; 
      //SimObj
      void setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg);
      void releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg);
      //Srv
      bool getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res);
      bool getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res);
      bool getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res);
      bool getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res);
      bool getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res);
      bool getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res);
      virtual ~SimObjController() = 0;
   protected:
      void initCommonTopicSvr();
      
   public:      
      SimObj *my;
      
      ros::Publisher onRecvMsg_pub;
      
      //Topic
      ros::Subscriber setJointVelocity_sub;
      ros::Subscriber releaseObj_sub;
      
      //Srv
      ros::ServiceServer service;
      ros::ServiceServer serviceGetObjPosition;
      ros::ServiceServer serviceGetPartsPosition;
      ros::ServiceServer serviceGetRotation;
      ros::ServiceServer serviceGetAngleRotation;
      ros::ServiceServer serviceGetJointAngle;
      
      //Robot
      double m_radius;           // radius of the wheel
	   double m_distance; 
	   
	   double m_simulatorTime;
};  

#endif
