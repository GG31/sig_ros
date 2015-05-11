#include "include/SimObjController.hpp"

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

bool SimObjController::graspObj(sig_ros::graspObj::Request &req, sig_ros::graspObj::Response &res)
{
   CParts *parts = my->getParts(req.part.c_str());
   res.ok = parts->graspObj(req.obj.c_str());
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
