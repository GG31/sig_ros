#include "include/GeneralController.hpp"


void GeneralController::onInit(InitEvent &evt) {
   /*std::cout << "GeneralController on init" << std::endl;
   int argc = 0;
   char** argv = NULL;
   myRobot = getRobotObj(myname());
   ros::init(argc, argv, std::string(this->myname()) + "_sig_controller_node");//+std::string(this->myname())*/
   RobotController::init();
   ros::NodeHandle n;
   setWheelVelocity_sub = n.subscribe<sig_ros::SetWheelVelocity>(std::string(this->myname()) + "_setWheelVelocity", 1, &GeneralController::setWheelVelocityCallback, this);
      
   std::cout << "fin init" << std::endl;
}

void GeneralController::setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel) {
   std::cout << "setWheelVelocityCallback" << std::endl;
   myRobot->setWheelVelocity(wheel->leftWheel, wheel->rightWheel);
}

extern "C"  GeneralController * createController ()
{
   return new GeneralController;
}
