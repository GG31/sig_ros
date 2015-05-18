#define CONTROLLER
#define dDOUBLE
#define USE_ODE
#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"

#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
//Msg
#include <sig_ros/MsgRecv.h>
#include <sig_ros/OnCollision.h>
#include <sig_ros/SetWheel.h>
#include <sig_ros/SetWheelVelocity.h>
#include <sig_ros/SetJointVelocity.h>
#include <sig_ros/ReleaseObj.h>
#include <sig_ros/SetRotation.h>
#include <sig_ros/Double3D.h>
//Srv
#include <sig_ros/getTime.h>
#include <sig_ros/getObjPosition.h>
#include <sig_ros/getPartsPosition.h>
#include <sig_ros/getRotation.h>
#include <sig_ros/getAngleRotation.h>
#include <sig_ros/getJointAngle.h>
#include <sig_ros/graspObj.h>
#include <sig_ros/getEntities.h>
#include <sig_ros/checkService.h>
#include <sig_ros/connectToService.h>
#include <sig_ros/getCollisionState.h>
#include <sig_ros/sendMsgToSrv.h>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )


//RARM_JOINT4 : avant-bras droit, + : derrière, - : devant
//RARM_JOINT1 : bras droit, + derrière, - : devant
class ModeratorCommand
{
   public:
      ModeratorCommand();
      void init();
      double loop(void);
      
   private:
      void getObjPosition(double l_tpos[], std::string obj);
      void getRotation(double rot[], std::string obj);
      double getJointAngle(std::string nameArm, std::string name);
      bool getCollisionState(std::string part, std::string name);
      bool sendMsgToSrv(std::string msg, std::string name);
      void setRotation(double qW, double qX, double qY, double qZ, std::string name);
      void setPosition(double x, double y, double z, std::string name);
      
      ros::ServiceClient serviceGetObjPosition;
	   sig_ros::getObjPosition srvGetObjPosition;
	   ros::ServiceClient serviceGetRotation;
	   sig_ros::getRotation srvGetRotation;
	   ros::ServiceClient serviceGetJointAngle;
	   sig_ros::getJointAngle srvGetJointAngle;
	   ros::ServiceClient serviceGetCollisionState;
	   sig_ros::getCollisionState srvGetCollisionState;
	   ros::ServiceClient serviceSendMsgToSrv;
	   sig_ros::sendMsgToSrv srvSendMsgToSrv;
	   
	   ros::Publisher robot_000_setRotation_pub;
      sig_ros::SetRotation msgSetRotation;
      ros::Publisher robot_000_setPosition_pub;
      sig_ros::Double3D msgSetPosition;
	   
      ros::NodeHandle n;
      
      bool m_ref;   // Referee service
      double retValue;      // Refresh rate of the modification
      bool   colState;      // Collision state
      bool  pcolState;      // Collision state
      std::string roboName; // Robot's name
      std::string mdName;   // Moderator's name

      const static unsigned int jtnum = 7;
      std::vector<std::string> jointName;
      double prv1JAng_r[jtnum];
      double crrJAng_r[jtnum];

      std::vector<std::string> m_entities;
      std::vector<std::string> m_entNames;
      int entNum;

      // position vectors and rotation matrices required to modify robot's behavior
      double crrPos[3];
      double prv1Pos[3];
      double crrRot[4];
      double prv1Rot[4];

      double rsLen;
};  

