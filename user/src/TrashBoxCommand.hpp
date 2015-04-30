#include "ros/ros.h"
#include "Logger.h" 
#include <unistd.h>
#include <sstream>

//Msg
#include <sig_ros/MsgRecv.h>
#include <sig_ros/OnCollision.h>
#include <sig_ros/SetWheel.h>
#include <sig_ros/SetWheelVelocity.h>
#include <sig_ros/SetJointVelocity.h>
#include <sig_ros/ReleaseObj.h>
//Srv
#include <sig_ros/getTime.h>
#include <sig_ros/getObjPosition.h>
#include <sig_ros/getPartsPosition.h>
#include <sig_ros/getRotation.h>
#include <sig_ros/getAngleRotation.h>
#include <sig_ros/getJointAngle.h>
#include <sig_ros/graspObj.h> 
//Srv Obj
#include <sig_ros/checkService.h> 
#include <sig_ros/connectToService.h> 
#include <sig_ros/getCollisionStateOfMainPart.h> 
#include <sig_ros/getEntities.h> 
#include <sig_ros/isGrasped.h> 
  
class TrashBoxCommand { 
   public:
      TrashBoxCommand();
      void init();
      double loop();
      
   private: 
      void onMsgRecvCallback(const sig_ros::MsgRecv::ConstPtr& msg);
      void onCollisionCallback(const sig_ros::OnCollision::ConstPtr& msg);
      void getObjPosition(double l_tpos[], std::string obj);
      double checkService(std::string nameJoint);
      double connectToService(std::string nameJoint);
      bool getCollisionStateOfMainPart();
      void getAllEntities(std::vector<std::string> entities);
      bool isGrasped(std::string entityName);
      ros::NodeHandle n;
      std::vector<std::string> m_entities;

      // Trash size
      double tboxSize_x, tboxSize_z;

      // The height direction of the extent that is entered dust (y-direction )
      double tboxMin_y, tboxMax_y;

      bool   colState;      // In collision
      double retValue;
      std::string roboName;
      
      bool m_ref;
      
      //Publisher      
      ros::Publisher robot_000_setJointVelocity_pub;
      sig_ros::SetJointVelocity msgSetJointVelocity;
      
      ros::Publisher robot_000_releaseObj_pub;
      sig_ros::ReleaseObj msgReleaseObj;
      
      ros::Subscriber robot_000_onRecvMsg_sub;
      ros::Subscriber robot_000_onCollision_sub;
      
      //Service
      ros::ServiceClient serviceGetTime;
	   sig_ros::getTime srvGetTime;
	   ros::ServiceClient serviceGetObjPosition;
	   sig_ros::getObjPosition srvGetObjPosition;
	   ros::ServiceClient serviceGetPartsPosition;
	   sig_ros::getPartsPosition srvGetPartsPosition;
	   ros::ServiceClient serviceGetRotation;
	   sig_ros::getRotation srvGetRotation;
	   ros::ServiceClient serviceGetAngleRotation;
	   sig_ros::getAngleRotation srvGetAngleRotation;
	   ros::ServiceClient serviceGetJointAngle;
	   sig_ros::getJointAngle srvGetJointAngle;
	   ros::ServiceClient serviceGraspObj;
	   sig_ros::graspObj srvGraspObj;
	   //Srv Obj
	   ros::ServiceClient serviceCheckService;
	   sig_ros::checkService srvCheckService;
	   ros::ServiceClient serviceConnectToService;
	   sig_ros::connectToService srvConnectToService;
	   ros::ServiceClient serviceGetCollisionStateOfMainPart;
	   sig_ros::getCollisionStateOfMainPart srvGetCollisionStateOfMainPart;
	   ros::ServiceClient serviceGetEntities;
	   sig_ros::getEntities srvGetEntities;
	   ros::ServiceClient serviceIsGrasped;
	   sig_ros::isGrasped srvIsGrasped;
}; 
