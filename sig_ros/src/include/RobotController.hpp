#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#define CONTROLLER
#define dDOUBLE
#define USE_ODE
//SIGverse
#include "SimObjController.hpp"
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
#include <sig_ros/OnCollision.h>
#include <sig_ros/SetWheel.h>
#include <sig_ros/SetWheelVelocity.h>
#include <sig_ros/SetJointVelocity.h>
#include <sig_ros/ReleaseObj.h>
//Srv
#include "sig_ros/getTime.h"
#include "sig_ros/getObjPosition.h"
#include "sig_ros/getPartsPosition.h"
#include "sig_ros/getRotation.h"
#include "sig_ros/getAngleRotation.h"
#include <sig_ros/getJointAngle.h>
#include <sig_ros/graspObj.h>

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
      //void setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel);
      void setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg);
      void releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg);
      //Srv
      bool getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res);
      bool getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res);
      bool getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res);
      bool getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res);
      bool getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res);
      bool getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res);
      bool graspObj(sig_ros::graspObj::Request &req, sig_ros::graspObj::Response &res);
   
   protected:
      virtual void init();
      
   public:
      RobotObj *myRobot;
      //ros::NodeHandle *n;
      ros::Publisher onRecvMsg_pub;
      ros::Publisher onCollision_pub;
      
      //Topic
      ros::Subscriber setWheel_sub;
      //ros::Subscriber setWheelVelocity_sub;
      ros::Subscriber setJointVelocity_sub;
      ros::Subscriber releaseObj_sub;
      
      //Srv
      ros::ServiceServer service;
      ros::ServiceServer serviceGetObjPosition;
      ros::ServiceServer serviceGetPartsPosition;
      ros::ServiceServer serviceGetRotation;
      ros::ServiceServer serviceGetAngleRotation;
      ros::ServiceServer serviceGetJointAngle;
      ros::ServiceServer serviceGraspObj;
      
      //Robot
      double m_radius;           // radius of the wheel
	   double m_distance; 
	   
	   double m_simulatorTime;
};  

#endif
