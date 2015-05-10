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

extern "C"  Controller * createController ()
{
   return new SimObjController;
}
