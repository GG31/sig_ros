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
#include <sig_ros/OnCollision.h>
#include <sig_ros/SetAxisAndAngle.h>
#include <sig_ros/SetPosition.h>
//Srv
#include "sig_ros/getTime.h"
#include "sig_ros/getObjPosition.h"
#include "sig_ros/getPartsPosition.h"
#include "sig_ros/getRotation.h"
#include "sig_ros/getAngleRotation.h"
#include <sig_ros/getJointAngle.h>
//Obj Srv
#include <sig_ros/checkService.h>
#include <sig_ros/connectToService.h>
#include <sig_ros/getCollisionStateOfMainPart.h>
#include <sig_ros/getEntities.h>
#include <sig_ros/isGrasped.h>
#include <sig_ros/sendMsgToSrv.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define ARY_SIZE(ARY) ( (int)(sizeof(ARY)/sizeof(ARY[0])) )


class SimObjController : public Controller
{
   public:
      void onInit(InitEvent &evt);
      double onAction(ActionEvent &evt);
      void onRecvMsg(RecvMsgEvent &evt);
      void onCollision(CollisionEvent &evt); 
      //SimObj
      void setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg);
      void setAxisAndAngleCallback(const sig_ros::SetAxisAndAngle::ConstPtr& msg);
      void setPositionCallback(const sig_ros::SetPosition::ConstPtr& msg);
      void releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg);
      //Srv
      bool getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res);
      bool getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res);
      bool getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res);
      bool getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res);
      bool getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res);
      bool getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res);
      //Obj Srv
      bool getCollisionStateOfMainPart(sig_ros::getCollisionStateOfMainPart::Request &req, sig_ros::getCollisionStateOfMainPart::Response &res);
      bool srvCheckService(sig_ros::checkService::Request &req, sig_ros::checkService::Response &res);
      bool srvConnectToService(sig_ros::connectToService::Request &req, sig_ros::connectToService::Response &res);
      bool getEntities(sig_ros::getEntities::Request &req, sig_ros::getEntities::Response &res);
      bool isGrasped(sig_ros::isGrasped::Request &req, sig_ros::isGrasped::Response &res);
      bool sendMsgToSrv(sig_ros::sendMsgToSrv::Request &req, sig_ros::sendMsgToSrv::Response &res);
      
   public:      
      SimObj *my;
      
      //BaseService *m_ref;
      std::map<std::string, BaseService*> m_ref;

      ros::Publisher onRecvMsg_pub;
      ros::Publisher onCollision_pub;
      
      //Topic
      ros::Subscriber setJointVelocity_sub;
      ros::Subscriber releaseObj_sub;
      ros::Subscriber setAxisAndAngle_sub;
      ros::Subscriber setPosition_sub;
      
      //Srv
      ros::ServiceServer service;
      ros::ServiceServer serviceGetObjPosition;
      ros::ServiceServer serviceGetPartsPosition;
      ros::ServiceServer serviceGetRotation;
      ros::ServiceServer serviceGetAngleRotation;
      ros::ServiceServer serviceGetJointAngle;
      ros::ServiceServer serviceCheckService;
      ros::ServiceServer serviceConnectToService;
      ros::ServiceServer serviceGetCollisionStateOfMainPart;
      ros::ServiceServer serviceGetEntities;
      ros::ServiceServer serviceIsGrasped;
      ros::ServiceServer serviceSendMsgToSrv;
      
      //Robot
      double m_radius;           // radius of the wheel
	   double m_distance; 
	   
	   double m_simulatorTime;
};  

#endif
