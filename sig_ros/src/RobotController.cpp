#include "include/RobotController.hpp"

 
void RobotController::onInit(InitEvent &evt)
{
   SimObjController::init();
   myRobot = getRobotObj(myname());
   ros::NodeHandle n;
   
   //Topics
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &RobotController::setWheelCallback, this);
   setWheelVelocity_sub = n.subscribe<sig_ros::SetWheelVelocity>(std::string(this->myname()) + "_setWheelVelocity", 1, &RobotController::setWheelVelocityCallback, this);
}

/*****************************Callback topic************************/

void RobotController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   m_radius = wheel->wheelRadius;           // radius of the wheel
	m_distance = wheel->wheelDistance; 
   myRobot->setWheel(m_radius, m_distance);
}

void RobotController::setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel)
{
   myRobot->setWheelVelocity(wheel->leftWheel, wheel->rightWheel);
}

/*****************************End callback topic************************/

extern "C"  RobotController * createController ()
{
   return new RobotController;
}
