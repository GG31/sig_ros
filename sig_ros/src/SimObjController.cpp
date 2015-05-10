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
   //More Topics
   setAccel_sub = n.subscribe<sig_ros::Double3D>(std::string(this->myname()) + "_setAccel", 1, &SimObjController::setAccelCallback, this);
   setAngularVelocity_sub = n.subscribe<sig_ros::Double3D>(std::string(this->myname()) + "_setAngularVelocity", 1, &SimObjController::setAngularVelocityCallback, this);
   setTorque_sub = n.subscribe<sig_ros::Double3D>(std::string(this->myname()) + "_setTorque", 1, &SimObjController::setTorqueCallback, this);
   setVelocity_sub = n.subscribe<sig_ros::Double3D>(std::string(this->myname()) + "_setVelocity", 1, &SimObjController::setVelocityCallback, this);
   setCollisionEnable_sub = n.subscribe<sig_ros::SetCollisionEnable>(std::string(this->myname()) + "_setCollisionEnable", 1, &SimObjController::setCollisionEnableCallback, this);
   setGravityMode_sub = n.subscribe<sig_ros::SetGravityMode>(std::string(this->myname()) + "_setGravityMode", 1, &SimObjController::setGravityModeCallback, this);
   setJointAngle_sub = n.subscribe<sig_ros::SetJointAngle>(std::string(this->myname()) + "_setJointAngle", 1, &SimObjController::setJointAngleCallback, this);
   setJointQuaternion_sub = n.subscribe<sig_ros::SetJointQuaternion>(std::string(this->myname()) + "_setJointQuaternion", 1, &SimObjController::setJointQuaternionCallback, this);
   setMass_sub = n.subscribe<sig_ros::SetMass>(std::string(this->myname()) + "_setMass", 1, &SimObjController::setMassCallback, this);
   //Srv
   service = n.advertiseService(std::string(this->myname()) + "_get_time", &SimObjController::getTime, this);
   serviceGetObjPosition = n.advertiseService(std::string(this->myname()) + "_get_obj_position", &SimObjController::getObjPosition, this);
   serviceGetPartsPosition = n.advertiseService(std::string(this->myname()) + "_get_parts_position", &SimObjController::getPartsPosition, this);
   serviceGetRotation = n.advertiseService(std::string(this->myname()) + "_get_rotation", &SimObjController::getRotation, this);
   serviceGetAngleRotation = n.advertiseService(std::string(this->myname()) + "_get_angle_rotation", &SimObjController::getAngleRotation, this);
   serviceGetJointAngle = n.advertiseService(std::string(this->myname()) + "_get_joint_angle", &SimObjController::getJointAngle, this);
   //Obj
   serviceGetCollisionState = n.advertiseService(std::string(this->myname()) + "_get_collision_state", &SimObjController::getCollisionState, this);
   serviceCheckService = n.advertiseService(std::string(this->myname()) + "_check_service", &SimObjController::srvCheckService, this);
   serviceConnectToService = n.advertiseService(std::string(this->myname()) + "_connect_to_service", &SimObjController::srvConnectToService, this);
   serviceGetEntities = n.advertiseService(std::string(this->myname()) + "_get_entities", &SimObjController::getEntities, this);
   serviceIsGrasped = n.advertiseService(std::string(this->myname()) + "_is_graped", &SimObjController::isGrasped, this);
   serviceSendMsgToSrv = n.advertiseService(std::string(this->myname()) + "_send_msg_to_srv", &SimObjController::sendMsgToSrv, this);
   //More Srv
   serviceGetAllJointAngles = n.advertiseService(std::string(this->myname()) + "_get_all_joint_angles", &SimObjController::getAllJointAngles, this);
   serviceGetJointPosition = n.advertiseService(std::string(this->myname()) + "_get_joint_position", &SimObjController::getJointPosition, this);
   serviceGetMass = n.advertiseService(std::string(this->myname()) + "_get_mass", &SimObjController::getMass, this);
   
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
   my->setJointVelocity(msg->jointName.c_str(), msg->angularVelocity, msg->max);
}

void SimObjController::releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg) {
   CParts *parts = my->getParts(msg->arm.c_str());
	// release grasping
	parts->releaseObj();
}

void SimObjController::setAxisAndAngleCallback(const sig_ros::SetAxisAndAngle::ConstPtr& msg) {
   if (msg->name == "") {
      my->setAxisAndAngle(msg->axisX, msg->axisY, msg->axisZ, msg->angle);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAxisAndAngle(msg->axisX, msg->axisY, msg->axisZ, msg->angle);
   }
}

void SimObjController::setPositionCallback(const sig_ros::SetPosition::ConstPtr& msg) {
   if (msg->name == "") {
      my->setPosition(msg->posX, msg->posY, msg->posZ);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setPosition(msg->posX, msg->posY, msg->posZ);
   }
}

void SimObjController::setAccelCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setAccel(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAccel(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setAngularVelocityCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setAngularVelocity(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAngularVelocity(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setTorqueCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setTorque(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setTorque(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setVelocityCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setLinearVelocity(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setLinearVelocity(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setCollisionEnableCallback(const sig_ros::SetCollisionEnable::ConstPtr& msg) {
   if (msg->name == "") {
      my->setCollisionEnable(msg->flag);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setCollisionEnable(msg->flag);
   }
}
void SimObjController::setGravityModeCallback(const sig_ros::SetGravityMode::ConstPtr& msg) {
   if (msg->name == "") {
      my->setGravityMode(msg->gravity);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setGravityMode(msg->gravity);
   }
}
void SimObjController::setJointAngleCallback(const sig_ros::SetJointAngle::ConstPtr& msg) {
   if (msg->name == "") {
      my->setJointAngle(msg->jointName.c_str(), msg->angle);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setJointAngle(msg->jointName.c_str(), msg->angle);
   }
}
void SimObjController::setJointQuaternionCallback(const sig_ros::SetJointQuaternion::ConstPtr& msg) {
   if (msg->name == "") {
      my->setJointQuaternion(msg->jointName.c_str(), msg->qW, msg->qX, msg->qY, msg->qZ, msg->offset); //TODO ajouter offset default
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setJointQuaternion(msg->jointName.c_str(), msg->qW, msg->qX, msg->qY, msg->qZ, msg->offset);
   }
}
void SimObjController::setMassCallback(const sig_ros::SetMass::ConstPtr& msg) {
   if (msg->name == "") {
      my->setMass(msg->mass);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setMass(msg->mass);
   }
}
/*****************************End callback topic************************/


/*******************************Srv***********************************/
bool SimObjController::getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res)
{
   res.time = m_simulatorTime;
   return true;
}

bool SimObjController::getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res) //OK
{
   Vector3d pos;
   if (req.name == "") {
      my->getPosition(pos);
   } else {
      SimObj *obj = getObj(req.name.c_str());
      if (obj == NULL) {
         return false;
      }
      obj->getPosition(pos);
   }
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool SimObjController::getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res) //OK
{
   Vector3d pos;
   //my->getParts(req.part.c_str());
   if (req.name == "") {
      my->getPartsPosition (pos, req.part.c_str());
   } else {
      SimObj *obj = getObj(req.name.c_str());
      if (obj == NULL) {
         return false;
      }
      obj->getPartsPosition(pos, req.part.c_str());
   }
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool SimObjController::getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res) //OK
{
   Rotation ownRotation;
   if (req.name == "") {
	   my->getRotation(ownRotation);
	} else {
	   SimObj *obj = getObj(req.name.c_str());
	   if (obj == NULL) {
         return false;
      }
      obj->getRotation(ownRotation);
	}
   res.qW = ownRotation.qw();
   res.qY = ownRotation.qy();
   res.qX = ownRotation.qx();
   res.qZ = ownRotation.qz();
   return true;
}

bool SimObjController::getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res) //OK
{
	Vector3d vector3d = Vector3d(req.x, req.y, req.z);
	if (req.axis == "z") { 
	   res.angle = vector3d.angle(Vector3d(0.0, 0.0, 1.0));
	} else if (req.axis == "y") {
	   res.angle = vector3d.angle(Vector3d(0.0, 1.0, 0.0));
	} else if (req.axis == "x") {
	   res.angle = vector3d.angle(Vector3d(1.0, 0.0, 0.0));
	} else {
	   std::cout << "Not x, y or z : " << req.axis.c_str() << std::endl;
	}
   return true;
}

bool SimObjController::getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res) //OK but always return 0...
{
   if (req.name == "") {
      res.angle = my->getJointAngle(req.nameArm.c_str());
   } else {
      SimObj *obj = getObj(req.name.c_str());
      if (obj == NULL) {
         return false;
      }
      res.angle = obj->getJointAngle(req.nameArm.c_str());
   }
   return true;
}

bool SimObjController::getCollisionState(sig_ros::getCollisionState::Request &req, sig_ros::getCollisionState::Response &res) //OK
{
   CParts *parts;
   if (req.name == "" && req.part == "main") {
      parts = my->getMainParts();
   } else if (req.name == "" && req.part != "main") {
      parts = my->getParts(req.part.c_str());
   } else {
      SimObj *obj = getObj(req.name.c_str());
      if (obj == NULL) {
         return false;
      }
      if (req.part == "main") {
         parts = obj->getMainParts();
      } else {
         parts = obj->getParts(req.part.c_str());
      }
   }
   if (parts != NULL) {
      res.collisionState = parts->getCollisionState();
      return true;
   }
   else
      return false;
}

bool SimObjController::srvCheckService(sig_ros::checkService::Request &req, sig_ros::checkService::Response &res) //TODO test
{
   res.connected = checkService(req.serviceName.c_str()); 
   return true;
}

bool SimObjController::srvConnectToService(sig_ros::connectToService::Request &req, sig_ros::connectToService::Response &res) { //TODO test
   std::string serviceName = req.serviceName.c_str();
   m_ref[serviceName] = connectToService(serviceName);//"RobocupReferee"
   if (m_ref[serviceName] != NULL)
      res.connected = true;
   else
      res.connected = false;
   return true;
}

bool SimObjController::getEntities(sig_ros::getEntities::Request &req, sig_ros::getEntities::Response &res) { //OK
   std::vector<std::string> m_entities;
   getAllEntities(m_entities);
   res.length = m_entities.size();
   for (int i = 0; i < res.length; i++)
   {
      res.entitiesNames.push_back(m_entities[i]);
   }
   return true;
}

bool SimObjController::isGrasped(sig_ros::isGrasped::Request &req, sig_ros::isGrasped::Response &res) { //OK
   if (req.name == "") {
      res.answer = my->getIsGrasped();
   } else {
      SimObj *ent = getObj(req.name.c_str());
      if (ent == NULL) {
         return false;
      }
      res.answer = ent->getIsGrasped();
   }
   return true;
}

bool SimObjController::sendMsgToSrv(sig_ros::sendMsgToSrv::Request &req, sig_ros::sendMsgToSrv::Response &res) { //TODO srv
   res.ok = m_ref[req.name.c_str()]->sendMsgToSrv(req.msg.c_str());
   return true;
}

bool SimObjController::getAllJointAngles(sig_ros::getAllJointAngles::Request &req, sig_ros::getAllJointAngles::Response &res) { //TODO ask for getAllJointAngles when my is not robot_000
   std::map<std::string, double> list;
   std::cout << "getAllJointAngles" << std::endl;
   if (req.name == "") {
      std::cout << "if getall" << std::endl;
      list = my->getAllJointAngles();
      std::cout << "after get all" << std::endl;
   } else {
      std::cout << "else" << std::endl;
      SimObj *ent = getObj(req.name.c_str());
      if (ent == NULL) {
         return false;
      }
      std::cout << "obj" << std::endl;
      list = ent->getAllJointAngles();
      std::cout << " list" << std::endl;
   }
   res.length = list.size();
   std::cout << "map" << std::endl;
   for (std::map<std::string, double>::iterator it=list.begin(); it!=list.end(); ++it) {
      res.jointName.push_back(it->first);
      res.angle.push_back(it->second);
   }
   return true;
}

bool SimObjController::getJointPosition(sig_ros::getJointPosition::Request &req, sig_ros::getJointPosition::Response &res) {//OK
   Vector3d vec;
   if (req.name == "") {
      my->getJointPosition(vec, req.jointName.c_str());
   } else {
      SimObj *ent = getObj(req.name.c_str());
      if (ent == NULL) {
         return false;
      }
      ent->getJointPosition(vec, req.jointName.c_str());
   }
   res.posX = vec.x();
   res.posY = vec.y();
   res.posZ = vec.z();
   return true;
}

bool SimObjController::getMass(sig_ros::getMass::Request &req, sig_ros::getMass::Response &res) {
   if (req.name == "") {
      res.mass = my->getMass();
   } else {
      SimObj *ent = getObj(req.name.c_str());
      if (ent == NULL) {
         return false;
      }
      res.mass = ent->getMass();
   }
   return true;
}
      
/*****************************End Srv*********************************/
extern "C"  Controller * createController ()
{
   return new SimObjController;
}
