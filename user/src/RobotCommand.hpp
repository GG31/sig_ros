#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
//Msg
#include <sig_ros/MsgRecv.h>
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

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class RobotCommand
{
   public:
      RobotCommand();
      void init();
      double loop(void);
      
   private:
      void onMsgRecvCallback(const sig_ros::MsgRecv::ConstPtr& msg);
      void stopRobotMove(void);
      void array_init(double tab[], double v1, double v2, double v3);
      double rotateTowardObj(double pos[]);
      void recognizeObjectPosition(double l_tpos[], std::string m_trashName2);
      double getTime();
      double goToObj(double pos[], double range);
      void getPartsPosition(double l_pos[], std::string partName);
      void neutralizeArms(double evt_time);
      double getJointAngle(std::string nameJoint);
      void setJointVelocity(std::string jointName, double angularVelocity, double max);
      void setWheelVelocity(double leftWheel, double rightWheel);
      double goGraspingObject(double pos[]);
      void prepareThrowing(double evt_time);
      void throwTrash();
      
      ros::NodeHandle n;
      int m_state;
      double refreshRateOnAction;

	   std::string m_trashName1;
	   std::string m_trashName2;
	   std::string m_graspObjectName;

	   std::string m_trashBoxName1;
	   std::string m_trashBoxName2;

	   double m_angularVelocity;  // rotation speed of the wheel
	   double m_jointVelocity;    // rotation speed around the joint
	   double m_radius;           // radius of the wheel
	   double m_distance;         // length of wheel-track
	   double m_movingSpeed;      // actual velocity of the moving robot

	   // times wasted for moving, adjusting or driving
	   double m_time;
	   double m_time1;
	   double m_time4;

	   // set positions;
	   double m_frontTrashBox1 [3]; // for recycle material
	   double m_frontTrashBox2 [3]; // for burnable material
	   double m_relayPoint1    [3];
	   double m_frontTrash1    [3];
	   double m_frontTrash2    [3];

	   // condition flag for grasping trash
	   bool m_grasp;

      // angular parameter used to put robot's hands down
      double thetaA;   
      
      //Publisher
      ros::Publisher robot_000_setWheel_pub;
      sig_ros::SetWheel msgSetWheel;
      
      ros::Publisher robot_000_setWheelVelocity_pub;
      sig_ros::SetWheelVelocity msgSetWheelVelocity;
      
      ros::Publisher robot_000_setJointVelocity_pub;
      sig_ros::SetJointVelocity msgSetJointVelocity;
      
      ros::Publisher robot_000_releaseObj_pub;
      sig_ros::ReleaseObj msgReleaseObj;
      
      ros::Subscriber robot_000_onRecvMsg_sub;
      
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
};  

