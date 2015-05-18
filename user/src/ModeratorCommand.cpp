#include "ModeratorCommand.hpp"

ModeratorCommand::ModeratorCommand() {
   init();
}  

void ModeratorCommand::init() {
   serviceGetObjPosition = n.serviceClient<sig_ros::getObjPosition>("robot_000_get_obj_position");
   serviceGetRotation = n.serviceClient<sig_ros::getRotation>("robot_000_get_rotation");
   serviceGetJointAngle = n.serviceClient<sig_ros::getJointAngle>("robot_000_get_joint_angle");
   serviceGetCollisionState = n.serviceClient<sig_ros::getCollisionState>("robot_000_get_collision_state");
   serviceSendMsgToSrv = n.serviceClient<sig_ros::sendMsgToSrv>("robot_000_send_msg_to_srv");
   
   
   robot_000_setRotation_pub = n.advertise<sig_ros::SetRotation>("robot_000_setRotation", 1000);
   robot_000_setPosition_pub = n.advertise<sig_ros::Double3D>("robot_000_setPosition", 1000);
   
   int i, cnt;

   retValue = 0.1;
   //retValue = 0.2;
   colState = false;
   pcolState = false;
   roboName = "robot_000";
   mdName   = "moderator_0";

   crrPos[0] = 0;
   crrPos[1] = 0;
   crrPos[2] = 0;
   prv1Pos[0] = 0;
   prv1Pos[1] = 0;
   prv1Pos[2] = 0; 
   m_ref = false; 
   //getAllEntities(m_entities);
   ros::ServiceClient service = n.serviceClient<sig_ros::getEntities>("robot_000_get_entities");
   sig_ros::getEntities srv;
	if (service.call(srv)) {
	    m_entities = srv.response.entitiesNames;
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_entities");
	}
   // select objects to be observed
   cnt = m_entities.size();
   for (i = 0; i < cnt; i++){
      if((m_entities[i] != mdName) &&
      (m_entities[i] != roboName) &&
      (m_entities[i] != "trashbox_0") &&
      (m_entities[i] != "trashbox_1") &&
      (m_entities[i] != "trashbox_2") &&
      (m_entities[i] != "petbottle_0") &&
      (m_entities[i] != "petbottle_1") &&
      (m_entities[i] != "petbottle_2") &&
      (m_entities[i] != "petbottle_3") &&
      (m_entities[i] != "petbottle_4") &&
      (m_entities[i] != "banana") &&
      (m_entities[i] != "chigarette") &&
      (m_entities[i] != "chocolate") &&
      (m_entities[i] != "mayonaise_0") &&
      (m_entities[i] != "mayonaise_1") &&
      (m_entities[i] != "mugcup") &&
      (m_entities[i] != "can_0") &&
      (m_entities[i] != "can_1") &&
      (m_entities[i] != "can_2") &&
      (m_entities[i] != "can_3") &&
      (m_entities[i] != "apple") &&
      (m_entities[i] != "clock") &&
      (m_entities[i] != "kettle") ){
         m_entNames.push_back(m_entities[i]);
      }
   }
   entNum = m_entNames.size();
}

double ModeratorCommand::loop() {
// check whether Referee service is available or not
   bool available = false;
   ros::ServiceClient service = n.serviceClient<sig_ros::checkService>("trashbox_0_check_service");
   sig_ros::checkService srv;
   srv.request.serviceName = "CleanUpReferee";
	if (service.call(srv)) {
	   available = srv.response.connected;
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_check_service");
	}
	
   if(!available && m_ref != false) m_ref = false;
   else if(available && m_ref == false){
      //m_ref = false;//connectToService("CleanUpReferee");
      ros::ServiceClient service = n.serviceClient<sig_ros::connectToService>("robot_000_connect_to_service");
      sig_ros::connectToService srv;
      srv.request.serviceName = "CleanUpReferee";
      if (service.call(srv)) {
         m_ref = srv.response.connected;
      }  else {
         ROS_ERROR("Failed to call service robot_000_connect_to_service");
      }
   }

   // get information about the robot and renew it
   //----------------------------------
   getObjPosition(crrPos, roboName.c_str());
   getRotation(crrRot, roboName.c_str());
   /*SimObj *r_my = getObj(roboName.c_str());
   // for body configuration
   
   r_my->getPosition(crrPos);
   r_my->getRotation(crrRot);

   Vector3d rsLenVec(prv1Pos.x()-crrPos.x(), prv1Pos.y()-crrPos.y(), prv1Pos.z()-crrPos.z());
   rsLen = rsLenVec.length();
*/
   double temp[3] = {prv1Pos[0] - crrPos[0], prv1Pos[1] - crrPos[1], prv1Pos[2] - crrPos[2]};
   double rsLen = sqrt(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2]);

   // 
   for (int k = 0; k < entNum; k++){
 //     SimObj* locObj = getObj(m_entNames[k].c_str());
  //    CParts *parts = locObj->getMainParts();
      bool state = getCollisionState("main", m_entNames[k].c_str());;

      if (state){
         colState=true;       // collided with main body of robot
         if (rsLen == 0.0) {
            //r_my->setRotation(prv1Rot);
            setRotation(prv1Rot[0], prv1Rot[1], prv1Rot[2], prv1Rot[3], "");
         } else {
            //r_my->setPosition(prv1Pos);
            setPosition(prv1Pos[0], prv1Pos[1], prv1Pos[2], "");
         }

         std::string msg = "CleanUpReferee/Collision with [" + m_entNames[k] + "]" "/-100";
         if (m_ref != false) {
            //m_ref->sendMsgToSrv(msg.c_str());
            sendMsgToSrv(msg.c_str(), "CleanUpReferee");
         }
         else {
            ROS_INFO("%s", msg.c_str());
            //LOG_MSG((msg.c_str()));
         }
         break;
      }
   }

   //  if(!colState && !pcolState){
   if (!colState) {
      prv1Pos[0] = crrPos[0];
      prv1Pos[1] = crrPos[1];
      prv1Pos[2] = crrPos[2];
      prv1Rot[0] = crrRot[0];
      prv1Rot[1] = crrRot[1];
      prv1Rot[2] = crrRot[2];
      prv1Rot[3] = crrRot[3];
      //colState=false;     // reset collision condition
   } else if (pcolState) {
      for (int i = 0; i < jtnum; i++) {
         prv1JAng_r[i] = crrJAng_r[i];
      }
      pcolState = false;
   } else {
      //Do nothing on "collided" condition
      //    LOG_MSG((colPtName.c_str()));
      colState = false;
   }

  return retValue;  
} 

void ModeratorCommand::getObjPosition(double l_tpos[], std::string obj) {
   srvGetObjPosition.request.name = obj;
   if (serviceGetObjPosition.call(srvGetObjPosition)) {
      l_tpos[0] = srvGetObjPosition.response.posX;
      l_tpos[1] = srvGetObjPosition.response.posY;
      l_tpos[2] = srvGetObjPosition.response.posZ; 
   } else {
      ROS_ERROR("Failed to call service robot_000_get_obj_position");
   }
}

void ModeratorCommand::getRotation(double rot[], std::string obj) {
	if (serviceGetRotation.call(srvGetRotation)) {
	   rot[0] = srvGetRotation.response.qX;
	   rot[1] = srvGetRotation.response.qY;
	   rot[2] = srvGetRotation.response.qZ;
	   rot[3] = srvGetRotation.response.qW;
	}  else {
	   ROS_ERROR("Failed to call service robot_0_get_rotation");
	}
}

double ModeratorCommand::getJointAngle(std::string nameArm, std::string name) {
   srvGetJointAngle.request.nameArm = nameArm;
   srvGetJointAngle.request.name = name;
	if (serviceGetJointAngle.call(srvGetJointAngle)) {
	   return srvGetJointAngle.response.angle;
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_joint_angle");
	}
	return 0.0;
}

bool ModeratorCommand::getCollisionState(std::string part, std::string name) {
   srvGetCollisionState.request.part = part;
   srvGetCollisionState.request.name = name;
	if (serviceGetCollisionState.call(srvGetCollisionState)) {
	   return srvGetCollisionState.response.collisionState;
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_collision_state");
	}
	return true;
}

bool ModeratorCommand::sendMsgToSrv(std::string msg, std::string name) {
   srvSendMsgToSrv.request.name = name;
   srvSendMsgToSrv.request.msg = msg;
	if (serviceSendMsgToSrv.call(srvSendMsgToSrv)) {
	   return srvSendMsgToSrv.response.ok;
	}  else {
	   ROS_ERROR("Failed to call service robot_000_send_msg_to_srv");
	}
	return false;
}

void ModeratorCommand::setRotation(double qX, double qY, double qZ, double qW, std::string name) {
   msgSetRotation.name = name;
   msgSetRotation.qW = qW;
   msgSetRotation.qX = qX;
   msgSetRotation.qY = qY;
   msgSetRotation.qZ = qZ;
   robot_000_setRotation_pub.publish(msgSetRotation);
}

void ModeratorCommand::setPosition(double x, double y, double z, std::string name) {
   msgSetPosition.name = name;
   msgSetPosition.x = x;
   msgSetPosition.y = y;
   msgSetPosition.z = z;
   robot_000_setPosition_pub.publish(msgSetPosition);
}
	
int main(int argc, char **argv)
{
   ros::init(argc, argv, "moderator_command");
	
	ModeratorCommand* moderatorCommand = new ModeratorCommand();
	double result = 0;
	
   while (ros::ok())//
   {
      result = moderatorCommand->loop();
      if (result == -1) {
         return 1;
      } else {
         ros::spinOnce();
         //loop_rate.sleep();
         ros::Duration(result).sleep();
      }
   }
   return 0;
}

