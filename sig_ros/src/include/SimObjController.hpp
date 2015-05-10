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
//More Msg
#include <sig_ros/Double3D.h>
#include <sig_ros/SetCollisionEnable.h>
#include <sig_ros/SetGravityMode.h>
#include <sig_ros/SetJointAngle.h>
#include <sig_ros/SetJointQuaternion.h>
#include <sig_ros/SetMass.h>

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
#include <sig_ros/getCollisionState.h>
#include <sig_ros/getEntities.h>
#include <sig_ros/isGrasped.h>
#include <sig_ros/sendMsgToSrv.h>
//More Srv
#include <sig_ros/getAllJointAngles.h>
#include <sig_ros/getJointPosition.h>
#include <sig_ros/getMass.h>

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
      //SimObj Msg
      void setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg);
      void setAxisAndAngleCallback(const sig_ros::SetAxisAndAngle::ConstPtr& msg);
      void setPositionCallback(const sig_ros::SetPosition::ConstPtr& msg);
      void releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg);
      //More Msg
      void setAccelCallback(const sig_ros::Double3D::ConstPtr& msg);
      void setAngularVelocityCallback(const sig_ros::Double3D::ConstPtr& msg);
      void setTorqueCallback(const sig_ros::Double3D::ConstPtr& msg);
      void setVelocityCallback(const sig_ros::Double3D::ConstPtr& msg);
      void setCollisionEnableCallback(const sig_ros::SetCollisionEnable::ConstPtr& msg);
      void setGravityModeCallback(const sig_ros::SetGravityMode::ConstPtr& msg);
      void setJointAngleCallback(const sig_ros::SetJointAngle::ConstPtr& msg);
      void setJointQuaternionCallback(const sig_ros::SetJointQuaternion::ConstPtr& msg);
      void setMassCallback(const sig_ros::SetMass::ConstPtr& msg);
      //Srv
      bool getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res);
      bool getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res);
      bool getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res);
      bool getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res);
      bool getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res);
      bool getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res);
      //Obj Srv
      bool getCollisionState(sig_ros::getCollisionState::Request &req, sig_ros::getCollisionState::Response &res);
      bool srvCheckService(sig_ros::checkService::Request &req, sig_ros::checkService::Response &res);
      bool srvConnectToService(sig_ros::connectToService::Request &req, sig_ros::connectToService::Response &res);
      bool getEntities(sig_ros::getEntities::Request &req, sig_ros::getEntities::Response &res);
      bool isGrasped(sig_ros::isGrasped::Request &req, sig_ros::isGrasped::Response &res);
      bool sendMsgToSrv(sig_ros::sendMsgToSrv::Request &req, sig_ros::sendMsgToSrv::Response &res);
      //More Srv
      bool getAllJointAngles(sig_ros::getAllJointAngles::Request &req, sig_ros::getAllJointAngles::Response &res);
      bool getJointPosition(sig_ros::getJointPosition::Request &req, sig_ros::getJointPosition::Response &res);
      bool getMass(sig_ros::getMass::Request &req, sig_ros::getMass::Response &res);
      
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
      //More topics
      ros::Subscriber setAccel_sub;
      ros::Subscriber setAngularVelocity_sub;
      ros::Subscriber setTorque_sub;
      ros::Subscriber setVelocity_sub;
      ros::Subscriber setCollisionEnable_sub;
      ros::Subscriber setGravityMode_sub;
      ros::Subscriber setJointAngle_sub;
      ros::Subscriber setJointQuaternion_sub;
      ros::Subscriber setMass_sub;
      
      //Srv
      ros::ServiceServer service;
      ros::ServiceServer serviceGetObjPosition;
      ros::ServiceServer serviceGetPartsPosition;
      ros::ServiceServer serviceGetRotation;
      ros::ServiceServer serviceGetAngleRotation;
      ros::ServiceServer serviceGetJointAngle;
      ros::ServiceServer serviceCheckService;
      ros::ServiceServer serviceConnectToService;
      ros::ServiceServer serviceGetCollisionState;
      ros::ServiceServer serviceGetEntities;
      ros::ServiceServer serviceIsGrasped;
      ros::ServiceServer serviceSendMsgToSrv;
      //More Srv
      ros::ServiceServer serviceGetAllJointAngles;
      ros::ServiceServer serviceGetJointPosition;
      ros::ServiceServer serviceGetMass;
      //Robot
      double m_radius;           // radius of the wheel
	   double m_distance; 
	   
	   double m_simulatorTime;
};  

#endif
