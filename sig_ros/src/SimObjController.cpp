#include "include/SimObjController.hpp"
  
void SimObjController::onInit(InitEvent &evt)
{
   int argc = 0;
   char** argv = NULL;
   //m_ref = NULL;
   
   my = getObj(myname());
   ros::init(argc, argv, std::string(this->myname()) + "_sig_controller_node");//+std::string(this->myname())
   ros::NodeHandle n;
   
   //Topics
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   onCollision_pub = n.advertise<sig_ros::OnCollision>(std::string(this->myname())+"_onCollisionMsg", 1000);
   setJointVelocity_sub = n.subscribe<sig_ros::SetJointVelocity>(std::string(this->myname()) + "_setJointVelocity", 1, &SimObjController::setJointVelocityCallback, this);
   releaseObj_sub = n.subscribe<sig_ros::ReleaseObj>(std::string(this->myname()) + "_releaseObj", 1, &SimObjController::releaseObjCallback, this);
   setAxisAndAngle_sub = n.subscribe<sig_ros::SetAxisAndAngle>(std::string(this->myname()) + "_setAxisAndAngle", 1, &SimObjController::setAxisAndAngleCallback, this);
   setPosition_sub = n.subscribe<sig_ros::SetPosition>(std::string(this->myname()) + "_setPosition", 1, &SimObjController::setPositionCallback, this);
   //Srv
   service = n.advertiseService(std::string(this->myname()) + "_get_time", &SimObjController::getTime, this);
   serviceGetObjPosition = n.advertiseService(std::string(this->myname()) + "_get_obj_position", &SimObjController::getObjPosition, this);
   serviceGetPartsPosition = n.advertiseService(std::string(this->myname()) + "_get_parts_position", &SimObjController::getPartsPosition, this);
   serviceGetRotation = n.advertiseService(std::string(this->myname()) + "_get_rotation", &SimObjController::getRotation, this);
   serviceGetAngleRotation = n.advertiseService(std::string(this->myname()) + "_get_angle_rotation", &SimObjController::getAngleRotation, this);
   serviceGetJointAngle = n.advertiseService(std::string(this->myname()) + "_get_joint_angle", &SimObjController::getJointAngle, this);
   //Obj
   serviceGetCollisionStateOfMainPart = n.advertiseService(std::string(this->myname()) + "_get_collision_state_of_main_part", &SimObjController::getCollisionStateOfMainPart, this);
   serviceCheckService = n.advertiseService(std::string(this->myname()) + "_check_service", &SimObjController::srvCheckService, this);
   serviceConnectToService = n.advertiseService(std::string(this->myname()) + "_connect_to_service", &SimObjController::srvConnectToService, this);
   serviceGetEntities = n.advertiseService(std::string(this->myname()) + "_get_entities", &SimObjController::getEntities, this);
   serviceSendMsgToSrv = n.advertiseService(std::string(this->myname()) + "_send_msg_to_srv", &SimObjController::sendMsgToSrv, this);
   
   m_simulatorTime = 0;
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
	// Get name of entity which is touched by the robot
	const std::vector<std::string> & with = evt.getWith();
	// Get parts of the robot which is touched by the entity
	const std::vector<std::string> & mparts = evt.getMyParts();
	sig_ros::OnCollision msg;
	for (int i=0; i < with.size(); i++) {
      msg.name = with[i];
      msg.part = mparts[i];
      onCollision_pub.publish(msg);
   }
}

/*****************************Callback topic************************/
void SimObjController::setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg)
{
   //my->addForce(0.0,0.0,500.0);
   my->setJointVelocity(msg->jointName.c_str(), msg->angularVelocity, msg->max);
}

void SimObjController::releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg) {
   CParts *parts = my->getParts(msg->arm.c_str());
	// release grasping
	parts->releaseObj();
}

void SimObjController::setAxisAndAngleCallback(const sig_ros::SetAxisAndAngle::ConstPtr& msg) {
   if (msg->name.c_str() == "") {
      my->setAxisAndAngle(msg->axisX, msg->axisY, msg->axisZ, msg->angle);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAxisAndAngle(msg->axisX, msg->axisY, msg->axisZ, msg->angle);
   }
}

void SimObjController::setPositionCallback(const sig_ros::SetPosition::ConstPtr& msg) {
   if (msg->name.c_str() == "") {
      my->setPosition(msg->posX, msg->posY, msg->posZ);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setPosition(msg->posX, msg->posY, msg->posZ);
   }
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
   Vector3d pos;
   if (req.name.c_str() == "") { //TODO test
      my->getPosition(pos);
   } else {
      SimObj *obj = getObj(req.name.c_str());
      obj->getPosition(pos);
   }
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
	   std::cout << "on z" << std::endl;
	   res.angle = vector3d.angle(Vector3d(0.0, 0.0, 1.0));
	} else if (req.axis == "y") {
	   std::cout << "on y" << std::endl;
	   res.angle = vector3d.angle(Vector3d(0.0, 1.0, 0.0));
	} else if (req.axis == "x") {
	   res.angle = vector3d.angle(Vector3d(1.0, 0.0, 0.0));
	} else {
	   std::cout << "Not x, y or z : " << req.axis.c_str() << std::endl;
	}
   return true;
}

bool SimObjController::getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res)
{
   res.angle = my->getJointAngle(req.nameArm.c_str());
   return true;
}

bool SimObjController::getCollisionStateOfMainPart(sig_ros::getCollisionStateOfMainPart::Request &req, sig_ros::getCollisionStateOfMainPart::Response &res)
{
   //res.angle = my->getJointAngle(req.nameArm.c_str());
   CParts *parts = my->getMainParts();
   res.collisionState = parts->getCollisionState();
   return true;
}

bool SimObjController::srvCheckService(sig_ros::checkService::Request &req, sig_ros::checkService::Response &res)
{
   res.connected = checkService(req.serviceName.c_str()); 
   return true;
}

bool SimObjController::srvConnectToService(sig_ros::connectToService::Request &req, sig_ros::connectToService::Response &res) {
   std::string serviceName = req.serviceName.c_str();
   m_ref[serviceName] = connectToService(serviceName);//"RobocupReferee"
   if (m_ref[serviceName] != NULL)
      res.connected = true;
   else
      res.connected = false;
   return true;
}

bool SimObjController::getEntities(sig_ros::getEntities::Request &req, sig_ros::getEntities::Response &res) {
   std::vector<std::string> m_entities;
   getAllEntities(m_entities);
   res.length = m_entities.size();
   for (int i = 0; i < res.length; i++)
   {
      res.entitiesNames.push_back(m_entities[i]);
   }
   return true;
}

bool SimObjController::isGrasped(sig_ros::isGrasped::Request &req, sig_ros::isGrasped::Response &res) {
   if (req.entityName == "") {
      res.answer = my->getIsGrasped();
   } else {
      SimObj *ent = getObj(req.entityName.c_str());
      res.answer = ent->getIsGrasped();
   }
   return true;
}

bool SimObjController::sendMsgToSrv(sig_ros::sendMsgToSrv::Request &req, sig_ros::sendMsgToSrv::Response &res) {
   res.ok = m_ref[req.name.c_str()]->sendMsgToSrv(req.msg.c_str());
   return true;
}

/*****************************End Srv*********************************/
extern "C"  Controller * createController ()
{
   return new SimObjController;
}
