#include "RobotController.hpp"
  
/*void SimObjController::onInit(InitEvent &evt)
{
   int argc = 0;
   char** argv = NULL;
   my = getObj(myname());
   ros::init(argc, argv, std::string(this->myname()) + "_sig_controller_node");//+std::string(this->myname())
   ros::NodeHandle n;
   
   //Topics
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   setJointVelocity_sub = n.subscribe<sig_ros::SetJointVelocity>(std::string(this->myname()) + "_setJointVelocity", 1, &RobotController::setJointVelocityCallback, this);
   releaseObj_sub = n.subscribe<sig_ros::ReleaseObj>(std::string(this->myname()) + "_releaseObj", 1, &RobotController::releaseObjCallback, this);
   
   //Srv
   service = n.advertiseService(std::string(this->myname()) + "_get_time", &RobotController::getTime, this);
   serviceGetObjPosition = n.advertiseService(std::string(this->myname()) + "_get_obj_position", &RobotController::getObjPosition, this);
   serviceGetPartsPosition = n.advertiseService(std::string(this->myname()) + "_get_parts_position", &RobotController::getPartsPosition, this);
   serviceGetRotation = n.advertiseService(std::string(this->myname()) + "_get_rotation", &RobotController::getRotation, this);
   serviceGetAngleRotation = n.advertiseService(std::string(this->myname()) + "_get_angle_rotation", &RobotController::getAngleRotation, this);
   serviceGetJointAngle = n.advertiseService(std::string(this->myname()) + "_get_joint_angle", &RobotController::getJointAngle, this);
   
   m_simulatorTime = 0;
}*/
void SimObjController::initCommonTopicSvr() {
  std::cout << "yolé" << std::endl; 
}

double SimObjController::onAction(ActionEvent &evt)
{
   ros::spinOnce();
   m_simulatorTime = evt.time();
   return 0.01;
}

void SimObjController::onRecvMsg(RecvMsgEvent &evt)
{
   sig_ros::MsgRecv msg;
   msg.sender = evt.getSender();
   msg.content = evt.getMsg();
   onRecvMsg_pub.publish(msg);
}

void SimObjController::onCollision(CollisionEvent &evt)
{
	
}

/*****************************Callback topic************************/
void SimObjController::setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg)
{
   std::cout << msg->jointName << " " << msg->angularVelocity << " " << msg->max << std::endl;
   //my->addForce(0.0,0.0,500.0);
   my->setJointVelocity(msg->jointName.c_str(), msg->angularVelocity, msg->max);
}

void SimObjController::releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg) {
   CParts *parts = my->getParts(msg->arm.c_str());
	// release grasping
	parts->releaseObj();
}
/*****************************End callback topic************************/


/*******************************Srv***********************************/
bool SimObjController::getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res)
{
   std::cout << "on getTime" << std::endl;
   res.time = m_simulatorTime;
   return true;
}

bool SimObjController::getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res)
{
   SimObj *obj = getObj(req.name.c_str());
   Vector3d pos;
   obj->getPosition(pos);
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool SimObjController::getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res)
{
   Vector3d pos;
   //my->getParts(req.part.c_str());
  
   my->getPartsPosition (pos, req.part.c_str());
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool SimObjController::getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res)
{
   Rotation ownRotation;
   std::cout << "on getRotation" << std::endl;
	my->getRotation(ownRotation);
	res.qW = 0.0;
	res.qY = 0.0;
	if (req.axis == "y") { 
	   std::cout << "it's y " << std::endl;
	   res.qW = ownRotation.qw();
	   res.qY = ownRotation.qy();
	   std::cout << res.qW << " : " << res.qY << std::endl;
	} else {
	   std::cout << "Not y : " << req.axis.c_str() << std::endl;
	}
   return true;
}

bool SimObjController::getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res)
{
   Rotation ownRotation;
	my->getRotation(ownRotation);
	Vector3d vector3d = Vector3d(req.x, req.y, req.z);
	if (req.axis == "z") { 
	   std::cout << "It's z" << std::endl;
	   res.angle = vector3d.angle(Vector3d(0.0, 0.0, 1.0));
	} else {
	   std::cout << "Not z : " << req.axis.c_str() << std::endl;
	}
   return true;
}

bool SimObjController::getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res)
{
   res.angle = my->getJointAngle(req.nameArm.c_str());
   return true;
}
/*****************************End Srv*********************************/
SimObjController::~SimObjController() {}
/*extern "C"  Controller * createController ()
{
   return new SimObjController;
}*/
