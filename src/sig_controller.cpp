#include "sig_controller.hpp"
  
void RobotController::onInit(InitEvent &evt)
{
   int argc = 0;
   char** argv = NULL;
   my = getRobotObj(myname());
   //my->setPosition(4.3, 5.5, 2.3);
   //my->setWheel(10.0, 10.0);
   ros::init(argc, argv, std::string(this->myname()) + "_sig_controller_node");//+std::string(this->myname())
   ros::NodeHandle n;
   
   //Topics
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &RobotController::setWheelCallback, this);
   setWheelVelocity_sub = n.subscribe<sig_ros::SetWheelVelocity>(std::string(this->myname()) + "_setWheelVelocity", 1, &RobotController::setWheelVelocityCallback, this);
   setJointVelocity_sub = n.subscribe<sig_ros::SetJointVelocity>(std::string(this->myname()) + "_setJointVelocity", 1, &RobotController::setJointVelocityCallback, this);
   
   //Srv
   service = n.advertiseService(std::string(this->myname()) + "_get_time", &RobotController::getTime, this);
   serviceGetObjPosition = n.advertiseService(std::string(this->myname()) + "_get_obj_position", &RobotController::getObjPosition, this);
}

double RobotController::onAction(ActionEvent &evt)
{
   ros::spinOnce();
   return 0.01;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
   sig_ros::MsgRecv msg;
   msg.sender = evt.getSender();
   msg.content = evt.getMsg();
   onRecvMsg_pub.publish(msg);
}

void RobotController::onCollision(CollisionEvent &evt)
{
	
}

/*****************************Callback topic************************/
void RobotController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   std::cout << "setWheelCallback" << std::endl;
   my->setWheel(wheel->wheelRadius, wheel->wheelDistance);
}

void RobotController::setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel)
{
   std::cout << "setWheelVelocityCallback" << std::endl;
   my->setWheelVelocity(wheel->leftWheel, wheel->rightWheel);
}

void RobotController::setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg)
{
   std::cout << msg->jointName << " " << msg->angularVelocity << " " << msg->max << std::endl;
   //my->addForce(0.0,0.0,500.0);
   my->setJointVelocity(msg->jointName.c_str(), msg->angularVelocity, msg->max);
}
/*****************************End callback topic************************/


/*******************************Srv***********************************/
bool RobotController::getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res)
{
   std::cout << "on getTime" << std::endl;
   res.time = getSimulationTime();
   return true;
}

bool RobotController::getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res)
{
   SimObj *obj = getObj(req.name.c_str());
   Vector3d pos;
   obj->getPosition(pos);
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool RobotController::getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res)
{
   Vector3d pos;
   //my->getParts(req.part.c_str());
  
   my->getPartsPosition (pos, req.part.c_str());
   /*res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();*/
   return true;
}

bool RobotController::getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res)
{
   Rotation ownRotation;
	my->getRotation(ownRotation);
	if (req.axis.c_str() == "y") { 
	   res.qW = ownRotation.qw();
	   res.qY = ownRotation.qy();
	}
   return true;
}

bool RobotController::getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res)
{
   Rotation ownRotation;
	my->getRotation(ownRotation);
	Vector3d vector3d = Vector3d(req.x, req.y, req.z);
	if (req.axis.c_str() == "z") { 
	   res.angle = vector3d.angle(Vector3d(0.0, 0.0, 1.0));
	}
   return true;
}
/*****************************End Srv*********************************/

extern "C"  Controller * createController ()
{
   return new RobotController;
}
