#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#define CONTROLLER
#define dDOUBLE
#define USE_ODE
//SIGverse
#include "SimObjController.hpp"
#include <sig_ros/SetWheel.h>
#include <sig_ros/SetWheelVelocity.h>

class RobotController : public SimObjController
{
   public:
      void onInit(InitEvent &evt);
   
   private:
      //Robot
      void setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel);
      void setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel);
      
      RobotObj *myRobot;
      //Topic
      ros::Subscriber setWheel_sub;
      ros::Subscriber setWheelVelocity_sub;
      //Robot
      double m_radius;           // radius of the wheel
	   double m_distance;
};  

#endif
