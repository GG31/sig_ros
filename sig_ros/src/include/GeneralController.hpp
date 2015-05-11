#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

//#define CONTROLLER
//#define dDOUBLE
//#define USE_ODE
//SIGverse
//#include "Controller.h"
#include "RobotController.hpp"



class GeneralController : public RobotController
{
   public:
      ros::Subscriber setWheelVelocity_sub;
      void setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel);
      void onInit(InitEvent &evt);
};  

#endif
